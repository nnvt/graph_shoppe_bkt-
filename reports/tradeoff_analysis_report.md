# Report Trade-off Giữa Các Phương Pháp

File này phân tích trade-off giữa bốn phương pháp trong bài toán giao hàng đa tác tử online:

- GreedyBFS
- ACO
- MAPD-CBS
- VRP-OrTools / VRP-style final

Phân tích dựa trên:

- code final trong `solvers/`;
- baseline trong `solvers/archive/`;
- benchmark mới trên `test_config.txt`;
- các metric: `net_reward`, `delivered`, `missed`, `late`, `on_time_rate`, `reward/delivered`, `runtime`.

## 1. Bảng tổng hợp kết quả

| Solver | Phiên bản | Net reward | Delivered | Missed | Late | On-time rate | Reward/delivered | Runtime |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| GreedyBFS | Baseline | 1447.98 | 70/320 | 250 | 12 | 82.86% | 20.69 | 1.47s |
| GreedyBFS | Final | 4081.92 | 234/320 | 86 | 43 | 81.62% | 17.44 | 0.76s |
| ACO | Baseline | 2435.91 | 121/320 | 199 | 20 | 83.47% | 20.13 | 15.96s |
| ACO | Final | 5621.15 | 293/320 | 27 | 31 | 89.42% | 19.18 | 124.29s |
| MAPD-CBS | Baseline | 5019.66 | 288/320 | 32 | 60 | 79.17% | 17.43 | 35.04s |
| MAPD-CBS | Final | 5708.37 | 295/320 | 25 | 35 | 88.14% | 19.35 | 33.45s |
| VRP-OrTools | Baseline | 5120.86 | 290/320 | 30 | 54 | 81.38% | 17.66 | 80.57s |
| VRP-OrTools | Final | 5794.99 | 296/320 | 24 | 22 | 92.57% | 19.58 | 1.65s |

Nhận xét tổng quan:

- GreedyBFS final nhanh nhất trong nhóm solver thuần heuristic, nhưng score thấp nhất trong các final.
- ACO final tăng throughput rất mạnh nhưng runtime cao nhất.
- MAPD-CBS final cân bằng tốt giữa reward, conflict handling và runtime.
- VRP-OrTools final đạt score cao nhất, on-time rate cao nhất và runtime rất thấp, nhưng final hiện là heuristic VRP-style chứ không còn trực tiếp gọi OR-Tools.

## 2. Trade-off theo throughput

Throughput đo bằng số đơn giao được và số đơn missed. Đây là yếu tố quyết định lớn nhất với các baseline yếu.

| Solver final | Delivered | Missed | Delivery rate |
|---|---:|---:|---:|
| GreedyBFS | 234/320 | 86 | 73.13% |
| ACO | 293/320 | 27 | 91.56% |
| MAPD-CBS | 295/320 | 25 | 92.19% |
| VRP-OrTools | 296/320 | 24 | 92.50% |

### GreedyBFS

GreedyBFS final cải thiện rất nhiều so với baseline:

```text
delivered: 70 -> 234
missed:    250 -> 86
```

Tuy nhiên throughput vẫn thấp hơn ba solver còn lại. Lý do chính:

- mỗi shipper chọn target theo heuristic cục bộ;
- không tối ưu route nhiều đơn;
- không có assignment toàn cục mạnh;
- không có cơ chế xử lý conflict nhiều bước;
- nếu chọn sai target ở map lớn, shipper có thể mất nhiều timestep để sửa.

Trade-off:

```text
rất nhanh, đơn giản
nhưng bỏ lỡ nhiều đơn hơn khi backlog lớn
```

### ACO

ACO final giao `293/320`, gần bằng MAPD-CBS và VRP. Cải thiện so với baseline rất lớn:

```text
delivered: 121 -> 293
missed:    199 -> 27
```

Lý do:

- nhiều ant thử nhiều phương án;
- mỗi shipper có thể được gán tập đơn/route ngắn hạn;
- route search tính gain khi thêm order;
- cooperative A* giúp giảm va chạm ở bước đi.

Trade-off:

```text
throughput cao
nhưng runtime tăng mạnh vì phải mô phỏng nhiều phương án
```

### MAPD-CBS

MAPD-CBS baseline đã có throughput cao (`288/320`), final tăng lên `295/320`. Điều này cho thấy với MAPD-CBS, điểm nghẽn chính không còn là missed mà là chất lượng deadline/reward.

Trade-off:

```text
giao nhiều đơn và kiểm soát conflict tốt
nhưng scoring assignment cần tốt để giảm late
```

### VRP-OrTools

VRP final giao nhiều nhất: `296/320`. Missed chỉ `24`. Khả năng route insertion giúp tận dụng capacity và lựa chọn shipper gần route hơn.

Trade-off:

```text
throughput cao nhất
nhưng thuật toán final dùng nhiều heuristic chuyên biệt hơn Greedy/ACO/MAPD
```

## 3. Trade-off theo deadline awareness

Deadline awareness được phản ánh qua `late` và `on_time_rate`.

| Solver final | Late | On-time rate |
|---|---:|---:|
| GreedyBFS | 43 | 81.62% |
| ACO | 31 | 89.42% |
| MAPD-CBS | 35 | 88.14% |
| VRP-OrTools | 22 | 92.57% |

### GreedyBFS

GreedyBFS late `43`, on-time rate `81.62%`. Đây là thấp nhất trong các solver final. Nguyên nhân:

- score pickup chủ yếu là reward trên tổng khoảng cách;
- không mô phỏng đầy đủ tác động của pickup mới lên các delivery đang mang;
- thiếu route-level deadline propagation.

GreedyBFS có thể giao nhiều hơn baseline nhưng phải trả giá bằng late tăng:

```text
late: 12 -> 43
```

Điều này hợp lý vì baseline giao quá ít đơn, còn final nhận thêm nhiều đơn khó hơn.

### ACO

ACO final late `31`, on-time rate `89.42%`. Đây là mức tốt vì ACO vừa tăng throughput vừa tăng on-time rate so với baseline:

```text
on_time_rate: 83.47% -> 89.42%
late:         20 -> 31
delivered:    121 -> 293
```

Late tăng về số tuyệt đối nhưng giảm tương đối vì số đơn giao tăng rất mạnh. Route search có tính reward đúng hạn/trễ nên deadline awareness tốt hơn GreedyBFS.

### MAPD-CBS

MAPD-CBS final late giảm mạnh:

```text
late: 60 -> 35
on_time_rate: 79.17% -> 88.14%
```

Điều này đúng với bản chất cải tiến: baseline đã giao được nhiều, final cải thiện assignment score, delivery grouping và detour penalty. Trade-off tốt nhất của MAPD-CBS là deadline và conflict cùng được xử lý tương đối cân bằng.

### VRP-OrTools

VRP final tốt nhất về deadline:

```text
late: 22
on_time_rate: 92.57%
```

Lý do:

- route evaluation dùng reward gần với môi trường;
- stale filter loại đơn quá trễ;
- route insertion kiểm tra feasibility;
- shipper idle được reposition về vùng nóng để giảm pickup delay.

Trade-off:

```text
deadline tốt nhất
nhưng code có nhiều heuristic và tham số thích nghi theo map size
```

## 4. Trade-off theo reward/delivered

Reward/delivered phản ánh chất lượng trung bình của đơn đã giao.

| Solver final | Reward/delivered |
|---|---:|
| GreedyBFS | 17.44 |
| ACO | 19.18 |
| MAPD-CBS | 19.35 |
| VRP-OrTools | 19.58 |

GreedyBFS thấp nhất vì solver tăng throughput bằng cách nhận thêm nhiều đơn nhưng chưa tối ưu tốt chất lượng route. ACO, MAPD-CBS và VRP đều gần nhau, trong đó VRP cao nhất.

Điểm đáng chú ý:

- Baseline GreedyBFS có reward/delivered `20.69`, cao hơn final `17.44`.
- Điều này không nghĩa baseline tốt hơn. Baseline chỉ giao `70` đơn, thường là các đơn dễ/điểm cao, nên trung bình cao nhưng tổng thấp.
- Trong bài này, tổng net reward quan trọng hơn reward trung bình nếu missed quá cao.

Trade-off cốt lõi:

```text
chọn ít đơn dễ -> reward/order cao nhưng total thấp
chọn nhiều đơn hơn -> reward/order có thể giảm nhưng total tăng
```

ACO, MAPD-CBS và VRP final đạt cân bằng tốt hơn: vừa giao nhiều vừa giữ reward/delivered khoảng `19`.

## 5. Trade-off theo runtime

| Solver final | Runtime |
|---|---:|
| GreedyBFS | 0.76s |
| VRP-OrTools | 1.65s |
| MAPD-CBS | 33.45s |
| ACO | 124.29s |

### GreedyBFS

GreedyBFS final nhanh nhất vì:

- chỉ dùng greedy target selection;
- distance đã được precompute/cached;
- không chạy metaheuristic;
- không chạy CBS đầy đủ.

Trade-off:

```text
runtime rất thấp
nhưng score thấp hơn vì thiếu route/global planning
```

### ACO

ACO final chậm nhất vì:

- precompute all-pairs shortest path;
- nhiều ant và iteration;
- route search branch-and-bound;
- cooperative A* cho nhiều shipper.

Runtime `124.29s` vẫn dưới giới hạn 60 phút, nhưng cao hơn đáng kể so với các phương pháp khác.

Trade-off:

```text
chất lượng cao hơn GreedyBFS
nhưng chi phí tính toán lớn
```

### MAPD-CBS

MAPD-CBS final runtime `33.45s`, hợp lý với việc có space-time BFS và conflict repair. So với ACO, MAPD-CBS nhanh hơn vì không mô phỏng nhiều ant; so với Greedy/VRP heuristic, chậm hơn vì phải kiểm tra conflict nhiều bước.

Trade-off:

```text
runtime trung bình
đổi lại có xử lý xung đột đa tác tử rõ ràng
```

### VRP-OrTools

VRP final runtime `1.65s`, thấp bất thường so với baseline OR-Tools `80.57s` vì final không còn trực tiếp gọi OR-Tools. Nó dùng route evaluation heuristic và SSSP cache.

Trade-off:

```text
rất nhanh và score cao
nhưng phải ghi rõ final là VRP-style heuristic, không phải OR-Tools exact model
```

## 6. Trade-off theo khả năng xử lý conflict

| Solver | Conflict handling |
|---|---|
| GreedyBFS | Rất nhẹ, chủ yếu phụ thuộc env và target riêng |
| ACO | Có cooperative A* |
| MAPD-CBS | Mạnh nhất, có bounded CBS |
| VRP-OrTools | Có coordinated local collision avoidance trong final; baseline có CBS |

### GreedyBFS

GreedyBFS không có CBS thật sự. Nếu nhiều shipper cùng hướng về một vùng, solver có thể bị môi trường ưu tiên id nhỏ hơn và shipper id lớn phải chờ hoặc đi kém hiệu quả.

### ACO

ACO final dùng cooperative A*, trong đó shipper được lập path theo thứ tự ưu tiên, và path của shipper trước trở thành constraint cho shipper sau. Cách này giảm va chạm nhưng không đầy đủ như CBS vì không search cây constraint nhiều nhánh.

### MAPD-CBS

MAPD-CBS là phương pháp xử lý conflict rõ nhất:

- lập path trong space-time;
- phát hiện vertex conflict;
- phát hiện edge conflict;
- thêm constraint;
- replan.

Đây là lý do MAPD-CBS phù hợp với yêu cầu “xử lý xung đột đa tác tử”.

### VRP-OrTools

VRP baseline có CBS sau khi OR-Tools gán target. VRP final có coordinated local collision avoidance dựa trên mô phỏng bước đi. Nó không phải CBS đầy đủ, nhưng đủ nhẹ để giữ runtime thấp.

Trade-off:

```text
CBS mạnh hơn nhưng chậm hơn
local avoidance nhanh hơn nhưng ít đảm bảo hơn
```

## 7. Trade-off theo surge/hotspot

| Solver | Surge/hotspot strategy |
|---|---|
| GreedyBFS | Centroid của 30 pickup gần nhất, proactive idle trên một số map |
| ACO | Centroid pickup history, centroid penalty, proactive idle |
| MAPD-CBS | Không có detector riêng; phản ứng qua assignment và CBS |
| VRP-OrTools | Heat map, density, surge detection, congestion-aware reposition |

### GreedyBFS và ACO

Hai solver này có chiến lược nhẹ:

```text
pickup history -> centroid -> idle reposition
```

Ưu điểm:

- đơn giản;
- không đọc config;
- phản ứng được nếu nhiều pickup tập trung.

Nhược điểm:

- không phát hiện surge rate;
- không có heat diffusion;
- không phân biệt hotspot mới và hotspot cũ tốt bằng heat map;
- GreedyBFS chỉ proactive idle trên một số map.

### MAPD-CBS

MAPD-CBS không có detector hotspot/surge rõ ràng. Điểm mạnh của nó là khi nhiều shipper bị kéo vào vùng backlog, CBS giúp giảm va chạm.

Trade-off:

```text
không dự đoán vùng nóng
nhưng xử lý tốt hơn khi shipper đã hội tụ và phát sinh conflict
```

### VRP-OrTools

VRP final có chiến lược đầy đủ nhất:

```text
new orders -> heat injection -> decay -> diffusion -> surge detection -> hotspot reposition
```

Đây là phương pháp tốt nhất để viết phần nâng cao trong report.

Trade-off:

```text
phản ứng hotspot/surge tốt
nhưng code phức tạp và có nhiều tham số heuristic
```

## 8. Trade-off theo mức độ tối ưu

| Solver | Mức độ tối ưu | Lý do |
|---|---|---|
| GreedyBFS | Heuristic cục bộ | Chọn target theo score hiện tại, không tối ưu route toàn cục |
| ACO | Metaheuristic / near-optimal trong rolling horizon nhỏ | Nhiều ant thử nhiều route nhưng giới hạn iteration/candidate |
| MAPD-CBS | Conflict-aware heuristic | CBS bounded horizon, assignment không optimal toàn cục |
| VRP-OrTools baseline | Heuristic/near-optimal cho bài toán con | OR-Tools có time limit và candidate pruning |
| VRP-OrTools final | Heuristic VRP-style | Route insertion/evaluation, không solve exact VRP |

Không có phương pháp nào optimal cho toàn bài toán gốc vì:

- bài toán online, không biết đơn tương lai;
- nhiều ràng buộc capacity/deadline/conflict;
- mỗi timestep chỉ thực hiện một bước rồi replan;
- runtime bị giới hạn.

Vì vậy, “tối ưu” trong báo cáo nên hiểu là tối ưu heuristic trong rolling horizon.

## 9. Phân tích theo từng config Phase 1

### C1-C2: config nhỏ

Ở C1-C2, hầu hết solver final đều đạt on-time cao. Khác biệt score không lớn vì số đơn ít:

| Solver final | C1 | C2 |
|---|---:|---:|
| GreedyBFS | 264.86 | 425.78 |
| ACO | 246.92 | 489.42 |
| MAPD-CBS | 260.96 | 465.48 |
| VRP-OrTools | 264.35 | 465.95 |

Trade-off:

- GreedyBFS đủ tốt trên config nhỏ vì ít backlog/conflict.
- ACO overhead không cần thiết ở C1.
- VRP/MAPD bắt đầu có lợi ở C2.

### C3-C4: config trung bình

| Solver final | C3 | C4 |
|---|---:|---:|
| GreedyBFS | 217.99 | 881.74 |
| ACO | 887.94 | 1028.49 |
| MAPD-CBS | 914.60 | 1023.87 |
| VRP-OrTools | 888.14 | 1046.49 |

C3 cho thấy điểm yếu của GreedyBFS: final chỉ đạt `217.99`, thấp hơn nhiều so với ACO/MAPD/VRP. Lý do có thể là target heuristic cục bộ không phù hợp map/config này. Đây là bằng chứng rằng GreedyBFS không ổn định khi cấu trúc map và backlog phức tạp hơn.

### C5-C6: config lớn

| Solver final | C5 | C6 |
|---|---:|---:|
| GreedyBFS | 1157.31 | 1134.24 |
| ACO | 1343.72 | 1624.67 |
| MAPD-CBS | 1372.28 | 1671.19 |
| VRP-OrTools | 1439.33 | 1690.72 |

Ở config lớn, route-level planning và deadline protection trở nên quan trọng. VRP-OrTools và MAPD-CBS vượt GreedyBFS rõ ràng. ACO cũng tốt nhưng runtime C6 cao (`87.12s`), phản ánh chi phí metaheuristic tăng theo kích thước bài toán.

## 10. Kết luận theo mục tiêu sử dụng

### Nếu cần baseline đơn giản, chạy nhanh

Chọn GreedyBFS.

Ưu điểm:

- dễ giải thích;
- runtime thấp;
- đúng bản chất BFS/greedy.

Nhược điểm:

- missed cao hơn;
- không ổn định trên một số config;
- thiếu conflict/route planning.

### Nếu cần ACO cao hơn GreedyBFS để đạt điểm nâng cao

Chọn ACO final.

Ưu điểm:

- net reward `5621.15`, cao hơn GreedyBFS final `4081.92`;
- throughput cao;
- có metaheuristic thật sự.

Nhược điểm:

- runtime cao nhất;
- nhạy tham số ant/iteration/candidate;
- khó giải thích hơn Greedy.

### Nếu cần xử lý xung đột đa tác tử rõ ràng

Chọn MAPD-CBS.

Ưu điểm:

- có CBS bounded horizon;
- giảm late tốt so với baseline;
- score cao và runtime vừa phải.

Nhược điểm:

- không có hotspot detector trực tiếp;
- assignment vẫn heuristic;
- CBS bị giới hạn horizon nên không optimal đầy đủ.

### Nếu cần kết quả Phase 1 tốt nhất

Chọn VRP-OrTools final.

Ưu điểm:

- net reward cao nhất `5794.99`;
- on-time rate cao nhất `92.57%`;
- runtime rất thấp `1.65s`;
- có chiến lược surge/hotspot tốt nhất.

Nhược điểm:

- final không trực tiếp dùng OR-Tools, cần viết trung thực là VRP-style heuristic;
- code nhiều heuristic và tham số hơn;
- mức độ “VRP + OR-Tools” nằm ở baseline/archive, còn final là tối ưu thực dụng theo môi trường online.

## 11. Đoạn tóm tắt để đưa vào báo cáo chính

```text
GreedyBFS có ưu điểm lớn nhất là đơn giản và chạy nhanh, nhưng trade-off là quyết định cục bộ nên missed còn cao và không xử lý xung đột sâu. ACO cải thiện mạnh throughput nhờ nhiều ant thử nhiều phương án route, đạt net reward cao hơn GreedyBFS như yêu cầu nâng cao, nhưng runtime lớn nhất. MAPD-CBS cân bằng tốt giữa reward và runtime, đồng thời là phương pháp xử lý xung đột đa tác tử rõ nhất nhờ space-time planning và conflict repair; điểm yếu là không có detector hotspot riêng. VRP-OrTools baseline mô hình hóa đúng bài toán VRP/PDP bằng OR-Tools nhưng runtime cao; phiên bản final giữ tư duy VRP-style bằng route insertion và heat-map repositioning, đạt net reward và on-time rate tốt nhất với runtime thấp. Nhìn chung, trade-off chính là: GreedyBFS tối ưu tốc độ, ACO tối ưu khả năng khám phá phương án, MAPD-CBS tối ưu phối hợp đa tác tử, còn VRP-style final tối ưu cân bằng giữa throughput, deadline và phản ứng hotspot.
```
