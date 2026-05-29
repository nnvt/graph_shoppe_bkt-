# Report Surge & Hotspot Strategy

File này mô tả chính xác các chiến lược ứng phó surge và hotspot đang có trong **code final** của nhóm. Phạm vi chỉ xét các solver hiện tại trong thư mục `solvers/`:

- `solvers/greedy_bfs.py`
- `solvers/aco_solver.py`
- `solvers/mapd_cbs_solver.py`
- `solvers/vrp_ortools.py`

## 1. Bối cảnh bài toán

Trong đề bài, đơn hàng sinh online theo thời gian. Solver chỉ nhìn thấy observation hiện tại, không được đọc trực tiếp file config hoặc private field của môi trường để biết trước surge/hotspot.

Surge là giai đoạn tốc độ sinh đơn tăng cao. Hotspot là vùng không gian có nhiều pickup tập trung gần nhau. Trong thực tế chấm bài, solver cần phản ứng với hai hiện tượng này thông qua dữ liệu quan sát được:

```text
new_order_ids
orders hiện tại
pickup positions của các đơn đã thấy
backlog active orders
shipper positions
```

Vì vậy, chiến lược hợp lệ không phải là đọc trước tham số surge/hotspot, mà là học hoặc suy luận từ luồng đơn đã xuất hiện.

## 2. Tổng quan theo solver

| Solver final | Có phát hiện surge trực tiếp? | Có xử lý hotspot trực tiếp? | Mức độ |
|---|---:|---:|---|
| `vrp_ortools.py` | Có | Có | Mạnh nhất |
| `aco_solver.py` | Không trực tiếp | Có gián tiếp bằng centroid pickup history | Trung bình/nhẹ |
| `greedy_bfs.py` | Không trực tiếp | Có gián tiếp bằng centroid pickup history | Nhẹ |
| `mapd_cbs_solver.py` | Không | Không rõ ràng; chủ yếu xử lý conflict/backlog | Gián tiếp |

Kết luận ngắn:

- Chiến lược surge/hotspot rõ nhất nằm trong `vrp_ortools.py`.
- `aco_solver.py` và `greedy_bfs.py` có cơ chế reposition về centroid của pickup gần đây.
- `mapd_cbs_solver.py` không có detector hotspot/surge thật sự, nhưng có assignment và CBS để giảm va chạm khi nhiều shipper cùng đi vào vùng đông đơn.

## 3. VRP-OrTools final: heat-map, surge detection và hotspot repositioning

### 3.1. Thành phần chính trong code

Trong `solvers/vrp_ortools.py`, solver có class:

```python
class ThermodynamicHeatEstimator:
```

Class này mô hình hóa bản đồ như một hệ nhiệt:

- đơn mới xuất hiện tạo nhiệt tại pickup;
- nhiệt giảm dần theo thời gian;
- nhiệt khuếch tán sang ô lân cận;
- vùng có nhiệt cao được xem là hotspot quan sát được;
- tốc độ đơn mới trong cửa sổ gần đây được dùng để phát hiện surge.

Các biến quan trọng:

```python
self.heat_map: Dict[Position, float] = {}
self.order_appearance_history: Dict[int, int] = {}
self.seen_order_ids = set()
self.surge_active = False
```

Ý nghĩa:

| Biến | Ý nghĩa |
|---|---|
| `heat_map` | Lưu độ nóng của từng ô, đại diện cho mật độ pickup gần đây |
| `order_appearance_history` | Lưu số đơn mới theo timestep để ước lượng tốc độ sinh đơn |
| `seen_order_ids` | Tránh cộng heat nhiều lần cho cùng một đơn |
| `surge_active` | Cờ cho biết tốc độ đơn mới gần đây cao hơn nền |

### 3.2. Heat injection: phát hiện vùng pickup tập trung

Mỗi timestep, solver gọi:

```python
self.estimator.update(current_t, orders, new_order_ids)
```

Trong `update`, với mỗi order mới:

```python
self.heat_map[(order.sx, order.sy)] =
    self.heat_map.get((order.sx, order.sy), 0.0) + 5.0 * order.p
```

Điều này nghĩa là:

- Nếu có nhiều đơn mới cùng xuất hiện gần một vùng, heat tại vùng đó tăng nhanh.
- Đơn priority cao tạo heat mạnh hơn vì nhân với `order.p`.
- Solver không cần biết hotspot thật trong config; hotspot được suy luận từ pickup của đơn đã reveal.

Đây là cơ chế hotspot detection trực tiếp nhất trong các solver final.

### 3.3. Heat decay: quên dần hotspot cũ

Trước khi cộng heat mới, solver làm nguội heat map:

```python
for pos in list(self.heat_map.keys()):
    self.heat_map[pos] *= 0.85
    if self.heat_map[pos] < 0.1:
        del self.heat_map[pos]
```

Ý nghĩa:

- Hotspot cũ không còn đơn mới sẽ tự mất ảnh hưởng.
- Solver không bị kéo mãi về một vùng đã hết đơn.
- Hệ số `0.85` giúp cân bằng giữa nhớ lịch sử gần và thích nghi với thay đổi mới.

### 3.4. Heat diffusion: mở rộng hotspot sang vùng lân cận

Sau heat injection, solver khuếch tán heat:

```python
diffuse_rate = 0.20
give = heat * diffuse_rate
keep = heat - give
share = give / len(neighbors)
```

Ý nghĩa:

- Nếu pickup tập trung quanh một tâm hotspot, không chỉ ô pickup chính có heat mà các ô lân cận cũng có heat.
- Điều này phù hợp với đề bài, vì hotspot được định nghĩa theo vùng Manhattan quanh tâm.
- Shipper idle có thể đi về vùng lân cận hotspot thay vì chỉ đúng ô pickup cụ thể.

### 3.5. Surge detection: phát hiện tốc độ sinh đơn tăng

Trong `ThermodynamicHeatEstimator.update`, solver lưu số đơn mới theo timestep:

```python
self.order_appearance_history[t] = new_count
```

Sau đó tính số đơn mới trong cửa sổ gần đây:

```python
window_size = 15
recent_orders = sum(
    self.order_appearance_history.get(tt, 0)
    for tt in range(max(0, t - window_size + 1), t + 1)
)
active_rate = recent_orders / window_size
base_rate = self.G / max(self.T, 1)
self.surge_active = active_rate >= 2.0 * base_rate
```

Mô hình này có nghĩa:

```text
Nếu tốc độ đơn mới trung bình trong 15 timestep gần nhất
>= 2 lần tốc độ nền G/T
=> xem là đang surge.
```

Đây là phát hiện surge trực tiếp từ observation. Solver không đọc `surge window` trong config, mà ước lượng qua `new_order_ids`.

### 3.6. Spatial density: ghi nhớ mật độ pickup đã xuất hiện

Ngoài heat map, solver còn có:

```python
self.spatial_density: Dict[Position, float] = {}
```

Mỗi đơn mới làm tăng density tại pickup:

```python
self.spatial_density[pos] = self.spatial_density.get(pos, 0.0) + 1.0
```

Khác với heat map, density không decay trong đoạn code hiện tại. Nó đóng vai trò bộ nhớ dài hơn về các vị trí từng có nhiều đơn.

Khi chọn hotspot cho shipper idle, solver dùng:

```python
density_bonus = 0.5 * self.spatial_density.get(pos, 0.0)
```

Tức là vị trí vừa có heat cao vừa từng có nhiều đơn sẽ hấp dẫn hơn.

### 3.7. Reposition shipper idle về hotspot

Sau khi lập route cho các shipper có đơn, solver xét nhóm shipper idle:

```python
idle_shippers = [s for s in shippers if not routes[s.id]]
```

Nếu có hot positions:

```python
hot_cells = sorted(self.estimator.heat_map.items(), key=lambda x: -x[1])
hot_positions = [pos for pos, h in hot_cells]
```

Mỗi shipper idle chọn hotspot tốt nhất theo score:

```python
score = (heat + density_bonus) / (
    1.0 + d + dispersion_factor * assigned_count[pos]
)
```

Trong đó:

| Thành phần | Ý nghĩa |
|---|---|
| `heat` | Mật độ đơn mới gần đây |
| `density_bonus` | Lịch sử pickup lâu hơn |
| `d` | Khoảng cách từ shipper đến hotspot |
| `assigned_count[pos]` | Số shipper idle đã được kéo về hotspot đó |
| `dispersion_factor` | Mức phạt khi nhiều shipper cùng chọn một hotspot |

Sau khi chọn hotspot:

```python
move = self._next_move(s.position, best_hotspot)
raw_moves[s.id] = (move, 0)
```

Tức là shipper idle không đứng yên mà chủ động di chuyển về vùng có khả năng sinh đơn cao.

### 3.8. Congestion-aware surge behavior

Điểm quan trọng nhất trong xử lý surge:

```python
dispersion_factor = 0.5 if self.estimator.surge_active else 3.0
```

Khi không surge:

```text
dispersion_factor = 3.0
```

Nếu nhiều shipper cùng chọn một hotspot, score bị phạt mạnh. Điều này giúp phân tán shipper, tránh tất cả dồn vào một vùng khi nhu cầu bình thường.

Khi surge:

```text
dispersion_factor = 0.5
```

Phạt congestion nhẹ hơn, nghĩa là solver cho phép nhiều shipper tập trung vào hotspot vì backlog đang tăng nhanh.

Đây là cơ chế thích nghi cao điểm rõ ràng:

```text
bình thường -> phân tán shipper
surge -> tập trung shipper hơn vào vùng nóng
```

### 3.9. Liên hệ với kết quả

VRP-OrTools final là solver có kết quả tốt nhất trên `test_config.txt`:

| Metric | Giá trị |
|---|---:|
| Net reward | 5794.99 |
| Delivered | 296/320 |
| Missed | 24 |
| Late | 22 |
| On-time rate | 92.57% |
| Runtime | 1.65s |

So với baseline VRP-OrTools:

| Metric | Baseline | Final | Thay đổi |
|---|---:|---:|---:|
| Net reward | 5120.86 | 5794.99 | +674.13 |
| Delivered | 290 | 296 | +6 |
| Missed | 30 | 24 | -6 |
| Late | 54 | 22 | -32 |
| On-time rate | 81.38% | 92.57% | +11.19 điểm % |
| Runtime | 80.57s | 1.65s | -78.92s |

Không thể khẳng định toàn bộ cải thiện đến từ heat map, vì final còn có route evaluation, stale filter và SSSP cache. Tuy nhiên, heat-map repositioning giúp giảm thời gian idle và kéo shipper về vùng có đơn, đặc biệt khi đơn tập trung theo hotspot.

## 4. ACO final: centroid pickup-history

### 4.1. Có phát hiện surge trực tiếp không?

`solvers/aco_solver.py` **không có surge detector trực tiếp** giống VRP-OrTools. Không có biến tương đương `surge_active`.

Tuy nhiên, ACO có phản ứng gián tiếp với hotspot bằng lịch sử pickup.

### 4.2. Pickup history và centroid

Trong `__init__`:

```python
self.pickup_history: List[Position] = []
```

Mỗi lần replan, solver cập nhật pickup history:

```python
for o in orders.values():
    pos = (o.sx, o.sy)
    if pos not in self.pickup_history:
        self.pickup_history.append(pos)
if len(self.pickup_history) > 30:
    self.pickup_history = self.pickup_history[-30:]
```

Sau đó tính centroid:

```python
raw_centroid = (
    sum(r for r, c in self.pickup_history) // len(self.pickup_history),
    sum(c for r, c in self.pickup_history) // len(self.pickup_history)
)
self.centroid = min(self.free_cells, key=lambda pos: self._distance(pos, raw_centroid))
```

Ý nghĩa:

- Nếu nhiều pickup gần nhau, centroid sẽ dịch về vùng đó.
- Vì chỉ giữ 30 pickup gần nhất, centroid phản ánh phân bố đơn tương đối gần đây.
- Nếu hotspot thay đổi, centroid cũng có thể dịch theo sau một thời gian.

### 4.3. Reposition shipper idle

Nếu shipper không có route và không còn đơn chưa gán:

```python
if self.proactive_idle:
    goals[s.id] = self.centroid
else:
    goals[s.id] = s.position
```

`proactive_idle` phụ thuộc kích thước map:

```python
elif self.env.N == 15:
    self.proactive_idle = True
elif self.env.N >= 20:
    self.proactive_idle = True
```

Tức là trên một số map lớn hơn, shipper idle sẽ không đứng yên mà đi về centroid pickup.

### 4.4. Centroid penalty trong route score

ACO final còn có:

```python
self.centroid_penalty_coef = 0.02
```

cho map `N >= 20`. Trong `_solve_routing`:

```python
c_penalty = c_penalty_coef * self._distance(curr_pos, self.centroid)
net_reward = accumulated_reward + accumulated_cost - c_penalty
```

Ý nghĩa:

- Route kết thúc quá xa centroid pickup sẽ bị trừ điểm.
- Điều này giữ shipper gần vùng có lịch sử đơn dày, giúp phản ứng tốt hơn khi hotspot tiếp tục sinh đơn.

### 4.5. Đánh giá mức độ

ACO final có xử lý hotspot **gián tiếp**:

```text
pickup history -> centroid -> idle reposition / route-end penalty
```

Nhưng không có:

- heat map;
- diffusion;
- surge rate detection;
- congestion-aware hotspot assignment.

Vì vậy trong báo cáo nên viết ACO là “có cơ chế reposition theo centroid pickup gần đây”, không nên viết là “phát hiện surge trực tiếp”.

## 5. GreedyBFS final: centroid pickup-history đơn giản

### 5.1. Có phát hiện surge trực tiếp không?

`solvers/greedy_bfs.py` **không phát hiện surge trực tiếp**. Solver không tính tốc độ đơn mới, không có `surge_active`, không có heat map.

### 5.2. Pickup history và centroid

GreedyBFS final có cấu trúc tương tự ACO:

```python
self.pickup_history: List[Position] = []
```

Khi replan:

```python
for o in orders.values():
    pos = (o.sx, o.sy)
    if pos not in self.pickup_history:
        self.pickup_history.append(pos)
if len(self.pickup_history) > 30:
    self.pickup_history = self.pickup_history[-30:]
```

Sau đó tính centroid:

```python
raw_centroid = (
    sum(r for r, c in self.pickup_history) // len(self.pickup_history),
    sum(c for r, c in self.pickup_history) // len(self.pickup_history)
)
self.centroid = min(self.free_cells, key=lambda pos: self._distance(pos, raw_centroid))
```

### 5.3. Proactive idle

Trong `__init__`:

```python
self.proactive_idle = (self.env.N in (15, 20))
```

Khi shipper không có route:

```python
if self.proactive_idle:
    goal = self.centroid
else:
    goal = shipper.position
```

Ý nghĩa:

- Trên map `N = 15` hoặc `N = 20`, shipper idle sẽ đi về centroid pickup.
- Trên các map khác, shipper idle có thể đứng yên nếu không có target.

### 5.4. Đánh giá mức độ

GreedyBFS có xử lý hotspot nhẹ nhất:

```text
pickup history -> centroid -> idle reposition trên một số map
```

Không có surge detection, heat map hay congestion-aware dispatch. Cơ chế này vẫn hợp lệ để mô tả trong báo cáo, nhưng nên ghi là “phản ứng gián tiếp với hotspot”.

## 6. MAPD-CBS final: xử lý cao điểm qua assignment và conflict, không có hotspot detector rõ ràng

### 6.1. Có phát hiện surge/hotspot trực tiếp không?

`solvers/mapd_cbs_solver.py` **không có surge detector trực tiếp** và cũng không có heat map/centroid pickup history.

Trong code có biến:

```python
hotspot_bias = 4.0 if order.p == 3 else 0.0
```

Tên biến là `hotspot_bias`, nhưng thực chất nó chỉ cộng bonus cho đơn priority 3. Nó không dựa trên:

- mật độ pickup;
- số đơn mới;
- vùng không gian;
- lịch sử pickup;
- tốc độ sinh đơn.

Vì vậy không nên trình bày `hotspot_bias` này như một detector hotspot thật sự.

### 6.2. Cách MAPD-CBS phản ứng gián tiếp với cao điểm

MAPD-CBS xử lý surge/hotspot theo hướng vận hành:

- Khi backlog tăng, assignment layer tạo nhiều candidate pickup/delivery.
- Candidate được score theo reward, priority, deadline, distance và detour.
- CBS xử lý vertex/edge conflict khi nhiều shipper cùng đi qua vùng hẹp.
- Nếu nhiều shipper bị kéo về cùng khu vực, conflict repair giúp giảm va chạm và đứng chặn.

Các thành phần liên quan:

```python
candidate_pairs = []
candidate_pairs.sort(key=lambda x: (x[0], -x[1]), reverse=True)
```

và CBS:

```python
horizon = min(18, max(6, 2 * int(obs.get("N", 10))))
```

Phần trace cũng ghi được một số chỉ báo phân tích:

```python
"active_orders"
"new_orders"
"idle_shippers"
"nearest_pickup_distance"
```

Nhưng trace chỉ phục vụ phân tích, không trực tiếp điều khiển policy surge/hotspot.

### 6.3. Đánh giá mức độ

MAPD-CBS không phát hiện hotspot/surge rõ ràng, nhưng có khả năng xử lý nút cổ chai cục bộ tốt hơn Greedy nhờ CBS:

```text
nhiều shipper cùng vào vùng đông đơn -> dễ conflict -> CBS phát hiện và sửa path
```

Trong báo cáo nên viết:

> MAPD-CBS không dùng detector surge/hotspot riêng. Thuật toán phản ứng gián tiếp với cao điểm bằng assignment theo backlog hiện tại và CBS để giảm xung đột khi nhiều shipper hội tụ về cùng vùng.

## 7. So sánh chiến lược

| Solver | Cơ chế quan sát | Cơ chế phản ứng | Điểm mạnh | Điểm yếu |
|---|---|---|---|---|
| GreedyBFS | 30 pickup gần nhất | Idle shipper đi về centroid trên một số map | Rất nhẹ, nhanh | Không phát hiện surge; chỉ reposition đơn giản |
| ACO | 30 pickup gần nhất | Centroid penalty và idle reposition | Giữ route gần vùng có pickup dày | Không có heat/diffusion/surge detector |
| MAPD-CBS | Backlog hiện tại, conflict path | Assignment + CBS conflict repair | Giảm tắc nghẽn khi nhiều shipper hội tụ | Không có hotspot detector riêng |
| VRP-OrTools | New orders, heat map, density, recent order rate | Hotspot reposition, surge-aware congestion factor | Đầy đủ nhất, phản ứng trực tiếp với surge/hotspot | Phức tạp hơn, nhiều heuristic |

## 8. Cách viết vào báo cáo chính

Đoạn mô tả phù hợp để đưa vào report:

```text
Nhóm không đọc trực tiếp tham số surge/hotspot từ config. Thay vào đó, các solver phản ứng qua observation online. VRP-OrTools final có cơ chế rõ nhất: mỗi đơn mới tạo heat tại pickup, heat decay và diffusion theo thời gian để ước lượng hotspot; tốc độ đơn mới trong cửa sổ 15 timestep được so với tốc độ nền G/T để phát hiện surge. Khi surge_active, solver giảm hệ số phân tán để cho phép nhiều shipper tập trung hơn vào vùng nóng; khi không surge, hệ số phân tán lớn hơn để tránh dồn shipper vào cùng một ô. ACO và GreedyBFS dùng centroid của 30 pickup gần nhất để reposition shipper idle về vùng có đơn tập trung. MAPD-CBS không có detector hotspot riêng, nhưng assignment và CBS giúp xử lý xung đột khi nhiều shipper cùng di chuyển vào khu vực backlog cao.
```

## 9. Kết luận

Trong code final, chỉ `vrp_ortools.py` có chiến lược surge/hotspot hoàn chỉnh gồm:

```text
heat injection -> decay -> diffusion -> surge detection -> congestion-aware repositioning
```

`aco_solver.py` và `greedy_bfs.py` có chiến lược nhẹ hơn:

```text
pickup history -> centroid -> idle reposition
```

`mapd_cbs_solver.py` chủ yếu xử lý cao điểm ở tầng path planning:

```text
assignment theo order hiện tại -> CBS tránh conflict
```

Do đó, khi trình bày phần nâng cao, nên lấy VRP-OrTools làm chiến lược chính, sau đó mô tả GreedyBFS/ACO là biến thể centroid đơn giản và MAPD-CBS là cơ chế giảm tắc nghẽn đa tác tử.
