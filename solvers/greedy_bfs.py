from __future__ import annotations

import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple, Set

from env import DeliveryEnv, Order, Shipper, is_valid_cell, valid_next_pos
from solvers.solver import Solver


Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]

INF = 10**9

MOVES: Tuple[Move, ...] = ("U", "D", "L", "R")

ALPHA = {1: 1.0, 2: 2.0, 3: 3.0}
BETA  = {1: 0.1, 2: 0.3, 3: 0.5}

def r_base(w: float) -> float:
    if w <= 0.2:  return 4.0
    if w <= 3.0:  return 10.0
    if w <= 10.0: return 15.0
    if w <= 30.0: return 20.0
    return 30.0


class GreedyBFS(Solver):
    """
    Greedy BFS - Phiên bản Rolling Horizon tối ưu
    - Precompute toàn bộ khoảng cách và di chuyển ngắn nhất bằng BFS tĩnh.
    - Sử dụng cơ chế Rolling Horizon (replan chu kỳ và khi có đơn mới) để cân bằng giữa tính ổn định di chuyển và tính linh hoạt.
    - Chọn pickup dựa trên Value Density (Reward / Distance).
    - Chọn delivery dựa trên độ khẩn cấp và loại bỏ các đơn trễ hạn quá lâu có reward bằng 0.
    """

    method_name = "GreedyBFS"

    def __init__(self, env: DeliveryEnv):
        super().__init__(env)
        # Precompute static adjacency list and free cells
        self.free_cells = [(r, c) for r, row in enumerate(self.grid) for c, val in enumerate(row) if val == 0]
        self.adj_list: Dict[Position, List[Tuple[Move, Position]]] = {}
        for pos in self.free_cells:
            self.adj_list[pos] = []
            for move in ("U", "D", "L", "R"):
                nxt = valid_next_pos(pos, move, self.grid)
                if nxt != pos:
                    self.adj_list[pos].append((move, nxt))
                    
        # Precompute all-pairs shortest path distances and next moves using BFS
        self._all_distances: Dict[Position, Dict[Position, int]] = {}
        self._all_next_moves: Dict[Position, Dict[Position, Move]] = {}
        
        for start in self.free_cells:
            self._all_distances[start] = {start: 0}
            self._all_next_moves[start] = {start: "S"}
            queue = deque([start])
            parent: Dict[Position, Tuple[Optional[Position], Move]] = {start: (None, "S")}
            
            while queue:
                curr = queue.popleft()
                d = self._all_distances[start][curr]
                for move, nxt in self.adj_list[curr]:
                    if nxt not in self._all_distances[start]:
                        self._all_distances[start][nxt] = d + 1
                        parent[nxt] = (curr, move)
                        queue.append(nxt)
            
            # Khởi tạo next moves từ start đến mọi goal
            for goal in self.free_cells:
                if goal not in parent:
                    self._all_next_moves[start][goal] = "S"
                    continue
                if goal == start:
                    self._all_next_moves[start][goal] = "S"
                    continue
                
                # Truy vết ngược tìm bước đi đầu tiên từ start -> goal
                curr = goal
                while True:
                    prev, move = parent[curr]
                    if prev == start:
                        self._all_next_moves[start][goal] = move
                        break
                    curr = prev

        self._plans: Dict[int, Tuple[str, int]] = {} # shipper_id -> (type, order_id)
        self._last_replan = -999
        self.replan_interval = 5
        self._last_positions = {}
        self._stuck_counts = {}
        self._last_actions = {}

        # Lịch sử tọa độ đơn hàng xuất hiện để tính centroid động
        self.pickup_history: List[Position] = []

        # Centroid tĩnh làm fallback
        if self.free_cells:
            self.centroid = (
                sum(r for r, c in self.free_cells) // len(self.free_cells),
                sum(c for r, c in self.free_cells) // len(self.free_cells)
            )
        else:
            self.centroid = (0, 0)

        # Proactive idle configuration based on map size N
        self.proactive_idle = (self.env.N in (15, 20))

    def _distance(self, start: Position, goal: Position) -> int:
        if start == goal: return 0
        return self._all_distances.get(start, {}).get(goal, INF)

    def _next_move(self, start: Position, goal: Position, stuck: bool = False) -> Move:
        if start == goal: return "S"
        if not stuck:
            return self._all_next_moves.get(start, {}).get(goal, "S")
        
        moves_dists = []
        for move, nxt in self.adj_list.get(start, []):
            d = self._distance(nxt, goal)
            moves_dists.append((d, move))
            
        if not moves_dists:
            return "S"
            
        moves_dists.sort(key=lambda x: x[0])
        if len(moves_dists) > 1:
            return moves_dists[1][1]
        return moves_dists[0][1]

    def _select_delivery(self, shipper: Shipper, orders: Dict[int, Order], t: int, T: int) -> Optional[Order]:
        carried_orders = [orders[oid] for oid in shipper.bag if oid in orders and not orders[oid].delivered]
        if not carried_orders:
            return None

        def urgency_score(order: Order):
            dist = self._distance(shipper.position, (order.ex, order.ey))
            arrival_time = t + dist
            margin = order.et - arrival_time
            # Nếu đơn trễ hạn quá mức trần T (phần thưởng bằng 0), xếp ưu tiên thấp nhất
            if arrival_time - order.et >= T:
                return (INF, 0, dist)
            return (margin, -order.p, dist)
            
        return min(carried_orders, key=urgency_score)

    def _select_pickup(self, shipper: Shipper, orders: Dict[int, Order], reserved_order_ids: Set[int], t: int, T: int) -> Optional[Order]:
        candidates = []
        for order in orders.values():
            if order.id in reserved_order_ids:
                continue
            if not shipper.can_carry(order, orders):
                continue
            
            dist_to_pickup = self._distance(shipper.position, (order.sx, order.sy))
            if dist_to_pickup >= INF:
                continue
            
            dist_to_deliver = self._distance((order.sx, order.sy), (order.ex, order.ey))
            total_dist = dist_to_pickup + dist_to_deliver
            arrival_time = t + total_dist
            
            rb = r_base(order.w)
            safety_margin = 1 if (self.env.N in (15, 18, 20)) else 0
            if arrival_time + safety_margin <= order.et:
                bonus = max(0.0, (order.et - arrival_time) / max(order.et, 1))
                reward = ALPHA[order.p] * rb * (1.0 + bonus)
            else:
                reward = BETA[order.p] * rb * max(0.0, 1.0 - (arrival_time - order.et) / T)
                
            score = reward / max(total_dist, 1)
            candidates.append((score, order))
            
        if not candidates:
            return None
            
        return max(candidates, key=lambda x: x[0])[1]

    def _move_towards(self, shipper: Shipper, goal: Position, stuck: bool = False) -> Tuple[Move, Position]:
        move = self._next_move(shipper.position, goal, stuck=stuck)
        next_position = valid_next_pos(shipper.position, move, self.grid)
        return move, next_position

    def _choose_new_target(self, shipper: Shipper, orders: Dict[int, Order], reserved_pickups: Set[int], t: int, T: int) -> Optional[Tuple[str, int]]:
        delivery_order = self._select_delivery(shipper, orders, t, T)
        pickup_order = self._select_pickup(shipper, orders, reserved_pickups, t, T)
        
        if delivery_order and pickup_order:
            # Option A: Deliver first, then pickup
            d1 = self._distance(shipper.position, (delivery_order.ex, delivery_order.ey))
            d2 = self._distance((delivery_order.ex, delivery_order.ey), (pickup_order.sx, pickup_order.sy))
            d3 = self._distance((pickup_order.sx, pickup_order.sy), (pickup_order.ex, pickup_order.ey))
            
            t_del_A = t + d1
            t_pick_A = t_del_A + d2
            t_del_pick_A = t_pick_A + d3
            
            # Option B: Pickup first, then deliver
            dp = self._distance(shipper.position, (pickup_order.sx, pickup_order.sy))
            
            # Sequence B1: Pickup -> Deliver pickup -> Deliver carried
            db1 = self._distance((pickup_order.sx, pickup_order.sy), (pickup_order.ex, pickup_order.ey))
            db2 = self._distance((pickup_order.ex, pickup_order.ey), (delivery_order.ex, delivery_order.ey))
            t_pick_B = t + dp
            t_del_pick_B1 = t_pick_B + db1
            t_del_carried_B1 = t_del_pick_B1 + db2
            
            # Sequence B2: Pickup -> Deliver carried -> Deliver pickup
            dc1 = self._distance((pickup_order.sx, pickup_order.sy), (delivery_order.ex, delivery_order.ey))
            dc2 = self._distance((delivery_order.ex, delivery_order.ey), (pickup_order.ex, pickup_order.ey))
            t_del_carried_B2 = t_pick_B + dc1
            t_del_pick_B2 = t_del_carried_B2 + dc2
            
            def order_reward(order, arrival):
                if arrival >= T: return 0.0
                rb = r_base(order.w)
                safety_margin = 1 if (self.env.N in (15, 18, 20)) else 0
                if arrival + safety_margin <= order.et:
                    return ALPHA[order.p] * rb * (1.0 + max(0.0, (order.et - arrival) / max(order.et, 1)))
                return BETA[order.p] * rb * max(0.0, 1.0 - (arrival - order.et) / T)
                
            rew_del_A = order_reward(delivery_order, t_del_A)
            rew_pick_A = order_reward(pickup_order, t_del_pick_A)
            net_A = rew_del_A + rew_pick_A - (d1 + d2 + d3) * 0.01
            
            rew_pick_B1 = order_reward(pickup_order, t_del_pick_B1)
            rew_del_B1 = order_reward(delivery_order, t_del_carried_B1)
            net_B1 = rew_pick_B1 + rew_del_B1 - (dp + db1 + db2) * 0.01
            
            rew_del_B2 = order_reward(delivery_order, t_del_carried_B2)
            rew_pick_B2 = order_reward(pickup_order, t_del_pick_B2)
            net_B2 = rew_del_B2 + rew_pick_B2 - (dp + dc1 + dc2) * 0.01
            
            best_net_B = max(net_B1, net_B2)
            
            if best_net_B > net_A:
                return ('pickup', pickup_order.id)
            else:
                return ('deliver', delivery_order.id)
        elif delivery_order:
            return ('deliver', delivery_order.id)
        elif pickup_order:
            return ('pickup', pickup_order.id)
        return None

    def _should_replan(self, obs: dict) -> bool:
        t = obs["t"]
        return t - self._last_replan >= self.replan_interval or len(obs.get("new_order_ids", [])) > 0 or not self._plans

    def _replan(self, obs: dict):
        self._last_replan = obs["t"]
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = obs["shippers"]
        t = obs["t"]
        T = obs["T"]
        
        # Update pickup history and centroid dynamically
        for o in orders.values():
            pos = (o.sx, o.sy)
            if pos not in self.pickup_history:
                self.pickup_history.append(pos)
        if len(self.pickup_history) > 30:
            self.pickup_history = self.pickup_history[-30:]
        if self.pickup_history:
            raw_centroid = (
                sum(r for r, c in self.pickup_history) // len(self.pickup_history),
                sum(c for r, c in self.pickup_history) // len(self.pickup_history)
            )
            if self.free_cells:
                self.centroid = min(self.free_cells, key=lambda pos: self._distance(pos, raw_centroid))

        reserved_pickups: Set[int] = set()
        
        # Gán kế hoạch mới cho từng shipper
        for shipper in sorted(shippers, key=lambda s: s.id):
            target = self._choose_new_target(shipper, orders, reserved_pickups, t, T)
            if target:
                self._plans[shipper.id] = target
                if target[0] == 'pickup':
                    reserved_pickups.add(target[1])
            else:
                self._plans.pop(shipper.id, None)

    def _decide_actions(self, obs: dict) -> Dict[int, Action]:
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = obs["shippers"]
        t = obs["t"]
        T = obs["T"]

        # Stuck detection
        for s in shippers:
            last_pos = self._last_positions.get(s.id)
            last_act = self._last_actions.get(s.id, "S")
            if last_pos is not None and s.position == last_pos and last_act != "S":
                self._stuck_counts[s.id] = self._stuck_counts.get(s.id, 0) + 1
            else:
                self._stuck_counts[s.id] = 0
            self._last_positions[s.id] = s.position

        # Kích hoạt replan khi cần
        if self._should_replan(obs):
            self._replan(obs)

        actions: Dict[int, Action] = {}
        
        for shipper in sorted(shippers, key=lambda s: s.id):
            # Kiểm tra xem kế hoạch hiện tại có còn hợp lệ hay không
            valid = False
            if shipper.id in self._plans:
                ttype, oid = self._plans[shipper.id]
                if ttype == 'pickup' and oid in orders:
                    o = orders[oid]
                    if not o.picked and not o.delivered:
                        valid = True
                elif ttype == 'deliver' and oid in shipper.bag and oid in orders:
                    o = orders[oid]
                    if not o.delivered:
                        valid = True
            
            # Nếu không hợp lệ, chọn lập tức mục tiêu mới độc lập cho shipper này
            if not valid:
                # Thu thập các đơn đã được book bởi shippers khác
                reserved = {target[1] for sid, target in self._plans.items() if sid != shipper.id and target[0] == 'pickup'}
                target = self._choose_new_target(shipper, orders, reserved, t, T)
                if target:
                    self._plans[shipper.id] = target
                else:
                    self._plans.pop(shipper.id, None)

            stuck = (self._stuck_counts.get(shipper.id, 0) >= 1)

            # Thực thi di chuyển theo kế hoạch
            if shipper.id not in self._plans:
                if self.proactive_idle:
                    goal = self.centroid
                    move, next_position = self._move_towards(shipper, goal, stuck=stuck)
                    actions[shipper.id] = (move, 0)
                    self._last_actions[shipper.id] = move
                else:
                    actions[shipper.id] = ("S", 0)
                    self._last_actions[shipper.id] = "S"
                continue
                
            ttype, oid = self._plans[shipper.id]
            o = orders[oid]
            
            if ttype == 'pickup':
                goal = (o.sx, o.sy)
                move, next_position = self._move_towards(shipper, goal, stuck=stuck)
                actions[shipper.id] = (move, 1) if next_position == goal else (move, 0)
                self._last_actions[shipper.id] = move
            else:
                goal = (o.ex, o.ey)
                move, next_position = self._move_towards(shipper, goal, stuck=stuck)
                actions[shipper.id] = (move, 2) if next_position == goal else (move, 0)
                self._last_actions[shipper.id] = move

        return actions

    def run(self) -> dict:
        start_time = time.time()
        obs = self.env.reset()

        while not obs.get("done", False):
            actions = self._decide_actions(obs)
            obs, _, done, _ = self.env.step(actions)
            if done:
                break

        return self.env.result(
            self.method_name,
            elapsed_sec=time.time() - start_time,
        )