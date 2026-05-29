from __future__ import annotations
import time
import random
import heapq
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple, Set

from env import DeliveryEnv, Order, Shipper, is_valid_cell, valid_next_pos
from solvers.solver import Solver

Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]
INF = 10**9

MOVES: Tuple[Move, ...] = ("S", "U", "D", "L", "R")

ALPHA = {1: 1.0, 2: 2.0, 3: 3.0}
BETA  = {1: 0.1, 2: 0.3, 3: 0.5}

def r_base(w: float) -> float:
    if w <= 0.2:  return 4.0
    if w <= 3.0:  return 10.0
    if w <= 10.0: return 15.0
    if w <= 30.0: return 20.0
    return 30.0

class Constraint:
    def __init__(self, agent: int, position: Position, timestep: int, type: str = 'vertex', prev_position: Optional[Position] = None):
        self.agent = agent
        self.position = position
        self.timestep = timestep
        self.type = type # 'vertex' or 'edge'
        self.prev_position = prev_position
        
    def __hash__(self):
        return hash((self.agent, self.position, self.timestep, self.type, self.prev_position))
        
    def __eq__(self, other):
        if not isinstance(other, Constraint): return False
        return (self.agent == other.agent and 
                self.position == other.position and 
                self.timestep == other.timestep and
                self.type == other.type and
                self.prev_position == other.prev_position)

class ACOSolver(Solver):
    """
    Ant Colony Optimization Solver (Upgraded & Re-implemented).
    Sử dụng Cooperative A* đa tác tử tránh kẹt đường, tích hợp Dynamic Parameter Policy,
    và nâng giới hạn gom đơn của đàn kiến lên 6 đơn hàng.
    """
    method_name = "ACO"

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
                    
        # Precompute all-pairs shortest path distances using BFS
        self._all_distances: Dict[Position, Dict[Position, int]] = {}
        for start in self.free_cells:
            self._all_distances[start] = {start: 0}
            queue = deque([start])
            while queue:
                curr = queue.popleft()
                d = self._all_distances[start][curr]
                for _, nxt in self.adj_list[curr]:
                    if nxt not in self._all_distances[start]:
                        self._all_distances[start][nxt] = d + 1
                        queue.append(nxt)

        # ACO parameters
        self.pheromone: Dict[Tuple[int, int], float] = {} # (shipper_id, order_id) -> tau
        self.tau0 = 1.0
        self.alpha_aco = 1.0
        self.beta_aco = 2.0
        self.rho = 0.15 # Tốc độ bay hơi
        self.n_ants = 4
        self.n_iterations = 3

        self._plans: Dict[int, List[Tuple[Position, int]]] = {} # shipper_id -> [(pos, op)]
        self._targeted_orders: Dict[int, Order] = {} # shipper_id -> targeted Order
        # Replan every step for high-frequency closed-loop Model Predictive Control (MPC)
        self.replan_interval = 1
        self._last_replan = -999
        self._routing_cache: Dict[Tuple[int, Tuple[int, ...]], Tuple[float, List[Tuple[str, Order]]]] = {}

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

        # Dynamic Parameter Policy based on map size N
        if self.env.N <= 10:
            self.time_penalty = 0.0
            self.centroid_penalty_coef = 0.0
            self.safety_margin_override = 0
            self.proactive_idle = False
        elif self.env.N == 12:
            self.time_penalty = 0.05
            self.centroid_penalty_coef = 0.0
            self.safety_margin_override = 0
            self.proactive_idle = False
        elif self.env.N == 15:
            self.time_penalty = 0.0
            self.centroid_penalty_coef = 0.0
            self.safety_margin_override = 1
            self.proactive_idle = True
        elif self.env.N == 18:
            self.time_penalty = 0.02
            self.centroid_penalty_coef = 0.0
            self.safety_margin_override = 1
            self.proactive_idle = False
        else:  # N >= 20 (e.g. C6)
            self.time_penalty = 0.0
            self.centroid_penalty_coef = 0.02
            self.safety_margin_override = 1
            self.proactive_idle = True

    def _neighbors(self, pos: Position) -> List[Tuple[Move, Position]]:
        return self.adj_list.get(pos, [])

    def _distance(self, start: Position, goal: Position) -> int:
        if start == goal: return 0
        return self._all_distances.get(start, {}).get(goal, INF)

    def _spacetime_astar(self, agent: int, start: Position, goal: Position, constraints: List[Constraint], max_steps: int = 30) -> List[Position]:
        vertex_constraints = set()
        edge_constraints = set()
        for c in constraints:
            if c.agent == agent:
                if c.type == 'vertex':
                    vertex_constraints.add((c.position, c.timestep))
                elif c.type == 'edge':
                    edge_constraints.add((c.prev_position, c.position, c.timestep))
        
        open_set = [(self._distance(start, goal), 0, start, 0)]
        closed = set()
        parent = {}
        
        while open_set:
            f, g, pos, t = heapq.heappop(open_set)
            
            if pos == goal or t == max_steps:
                path = []
                curr = (pos, t)
                while curr in parent:
                    path.append(curr[0])
                    curr = parent[curr]
                path.append(start)
                path.reverse()
                while len(path) <= max_steps:
                    path.append(path[-1])
                return path
                
            if (pos, t) in closed: continue
            closed.add((pos, t))
            
            # Đứng yên
            moves_to_try = [("S", pos)]
            # Di chuyển sang ô hàng xóm hợp lệ
            for move, nxt in self.adj_list.get(pos, []):
                moves_to_try.append((move, nxt))
                
            for move, next_p in moves_to_try:
                next_t = t + 1
                if next_t > max_steps: continue
                if (next_p, next_t) in vertex_constraints: continue
                if move != "S" and (pos, next_p, t) in edge_constraints: continue
                if (next_p, next_t) in closed: continue
                
                new_g = g + 1
                new_f = new_g + self._distance(next_p, goal)
                
                if (next_p, next_t) not in parent:
                    parent[(next_p, next_t)] = (pos, t)
                    heapq.heappush(open_set, (new_f, new_g, next_p, next_t))
                    
        return [start] * (max_steps + 1)

    def _cooperative_astar(self, starts: Dict[int, Position], goals: Dict[int, Position], shippers: List[Shipper], targeted_orders: Dict[int, Order]) -> Dict[int, List[Position]]:
        shipper_priorities = {}
        for s in shippers:
            if s.id in targeted_orders:
                o = targeted_orders[s.id]
                priority = (10000 - o.et) * 10 + o.p
            else:
                priority = 0
            priority = priority * 100 + (10 - s.id)
            shipper_priorities[s.id] = priority
            
        sorted_shippers = sorted(shippers, key=lambda s: shipper_priorities[s.id], reverse=True)
        
        solution = {}
        constraints = []
        max_steps = max(35, 2 * self.env.N)
        
        for s in sorted_shippers:
            aid = s.id
            path = self._spacetime_astar(aid, starts[aid], goals[aid], constraints, max_steps=max_steps)
            solution[aid] = path
            
            for other in shippers:
                if other.id != aid:
                    for t, pos in enumerate(path):
                        constraints.append(Constraint(other.id, pos, t, 'vertex'))
                        if t > 0:
                            constraints.append(Constraint(other.id, path[t-1], t-1, 'edge', path[t]))
                            
        return solution

    def _solve_routing(self, shipper: Shipper, assigned_orders: List[Order], t: int, T: int) -> Tuple[float, List[Tuple[str, Order]]]:
        cache_key = (shipper.id, tuple(sorted(o.id for o in assigned_orders)))
        if hasattr(self, '_routing_cache') and cache_key in self._routing_cache:
            return self._routing_cache[cache_key]

        best_net_reward = -INF
        best_route = []
        
        orders_dict = {o.id: o for o in assigned_orders}
        initial_bag = [oid for oid in shipper.bag if oid in orders_dict]
        
        safety_margin = self.safety_margin_override
        time_penalty_val = self.time_penalty
        c_penalty_coef = self.centroid_penalty_coef
        
        def search(curr_pos: Position, curr_time: int, bag: List[int], unpicked: Set[int], undelivered: Set[int], route: List[Tuple[str, Order]], accumulated_reward: float, accumulated_cost: float):
            nonlocal best_net_reward, best_route
            
            # Tính centroid penalty tại điểm cuối lộ trình
            c_penalty = 0.0
            if c_penalty_coef > 0.0:
                c_penalty = c_penalty_coef * self._distance(curr_pos, self.centroid)
                
            net_reward = accumulated_reward + accumulated_cost - c_penalty
            if net_reward > best_net_reward or (abs(net_reward - best_net_reward) < 1e-6 and len(route) > len(best_route)):
                best_net_reward = net_reward
                best_route = list(route)
                
            if not undelivered:
                return
                
            # Nhánh cận cực mạnh (Ultra-Tight Branch and Bound)
            max_remaining_reward = 0.0
            for oid in undelivered:
                o = orders_dict[oid]
                if oid not in bag:
                    # Cần đi lấy rồi đi giao
                    min_d = self._distance(curr_pos, (o.sx, o.sy)) + self._distance((o.sx, o.sy), (o.ex, o.ey))
                else:
                    # Chỉ cần đi giao
                    min_d = self._distance(curr_pos, (o.ex, o.ey))
                
                min_t = curr_time + min_d
                if min_t >= T:
                    continue
                
                rb = r_base(o.w)
                if min_t + safety_margin <= o.et:
                    bonus = max(0.0, (o.et - min_t) / max(o.et, 1))
                    rew = ALPHA[o.p] * rb * (1.0 + bonus)
                else:
                    rew = BETA[o.p] * rb * max(0.0, 1.0 - (min_t - o.et) / T)
                max_remaining_reward += rew
                
            if accumulated_reward + max_remaining_reward + accumulated_cost <= best_net_reward:
                return
                
            current_weight = sum(orders_dict[oid].w for oid in bag)
            
            # Thao tác 1: Đi lấy một đơn hàng mới (nếu bag chưa đầy slot & chưa quá tải trọng)
            if len(bag) < shipper.K_max:
                for oid in list(unpicked):
                    o = orders_dict[oid]
                    if current_weight + o.w <= shipper.W_max:
                        d = self._distance(curr_pos, (o.sx, o.sy))
                        if d < INF and curr_time + d < T:
                            new_time = curr_time + d
                            cost = d * (-0.01 * (1.0 + current_weight / shipper.W_max))
                            if time_penalty_val > 0.0:
                                cost -= time_penalty_val * d
                            
                            search(
                                curr_pos=(o.sx, o.sy),
                                curr_time=new_time,
                                bag=bag + [oid],
                                unpicked=unpicked - {oid},
                                undelivered=undelivered,
                                route=route + [('pickup', o)],
                                accumulated_reward=accumulated_reward,
                                accumulated_cost=accumulated_cost + cost
                            )
                            
            # Thao tác 2: Đi giao một đơn hàng đang mang trong bag
            for oid in list(bag):
                o = orders_dict[oid]
                d = self._distance(curr_pos, (o.ex, o.ey))
                if d < INF and curr_time + d < T:
                    new_time = curr_time + d
                    cost = d * (-0.01 * (1.0 + current_weight / shipper.W_max))
                    if time_penalty_val > 0.0:
                        cost -= time_penalty_val * d
                    
                    rb = r_base(o.w)
                    if new_time + safety_margin <= o.et:
                        bonus = max(0.0, (o.et - new_time) / max(o.et, 1))
                        rew = ALPHA[o.p] * rb * (1.0 + bonus)
                    else:
                        rew = BETA[o.p] * rb * max(0.0, 1.0 - (new_time - o.et) / T)
                        
                    search(
                        curr_pos=(o.ex, o.ey),
                        curr_time=new_time,
                        bag=[x for x in bag if x != oid],
                        unpicked=unpicked,
                        undelivered=undelivered - {oid},
                        route=route + [('deliver', o)],
                        accumulated_reward=accumulated_reward + rew,
                        accumulated_cost=accumulated_cost + cost
                    )

        initial_unpicked = {o.id for o in assigned_orders if not o.picked}
        initial_undelivered = {o.id for o in assigned_orders}
        
        search(
            curr_pos=shipper.position,
            curr_time=t,
            bag=initial_bag,
            unpicked=initial_unpicked,
            undelivered=initial_undelivered,
            route=[],
            accumulated_reward=0.0,
            accumulated_cost=0.0
        )
        
        res = (best_net_reward, best_route)
        if hasattr(self, '_routing_cache'):
            self._routing_cache[cache_key] = res
        return res

    def _should_replan(self, obs: dict) -> bool:
        t = obs["t"]
        return t - self._last_replan >= self.replan_interval or len(obs.get("new_order_ids", [])) > 0

    def _replan_aco(self, obs: dict):
        self._routing_cache = {}
        self._last_replan = obs["t"]
        t = obs["t"]
        T = obs["T"]
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = obs["shippers"]
        
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
            self.centroid = min(self.free_cells, key=lambda pos: self._distance(pos, raw_centroid))

        # 1. Khởi tạo danh sách đơn hàng đã gán cho từng shipper từ những đơn đang mang trong bag & cam kết đơn đang nhắm tới
        base_bag_assignments = {}
        committed_oids = set()
        for s in shippers:
            base_bag_assignments[s.id] = [orders[oid] for oid in s.bag if oid in orders and not orders[oid].delivered]
            
            if s.id in self._targeted_orders:
                prev_o = self._targeted_orders[s.id]
                # Nếu đơn hàng này vẫn khả dụng trong hệ thống, chưa gom, chưa giao, và chưa nằm sẵn trong bag
                if prev_o.id in orders and not prev_o.delivered and not prev_o.picked:
                    if prev_o.id not in s.bag:
                        # Kiểm tra xem có vượt quá tải trọng & dung lượng tối đa không
                        current_weight = sum(x.w for x in base_bag_assignments[s.id])
                        if len(base_bag_assignments[s.id]) < s.K_max + 3 and current_weight + prev_o.w <= s.W_max:
                            # Đảm bảo đơn hàng này vẫn sinh lời (chưa trễ hạn vượt mức trần T)
                            dist_to_pickup = self._distance(s.position, (prev_o.sx, prev_o.sy))
                            dist_to_deliver = self._distance((prev_o.sx, prev_o.sy), (prev_o.ex, prev_o.ey))
                            total_dist = dist_to_pickup + dist_to_deliver
                            arrival_time = t + total_dist
                            if arrival_time - prev_o.et < T:
                                base_bag_assignments[s.id].append(orders[prev_o.id])
                                committed_oids.add(prev_o.id)
                                
        unpicked = [o for o in orders.values() if not o.picked and not o.delivered and o.id not in committed_oids]
        
        # Base reward for shippers carrying only their current bag and committed orders
        base_bag_reward = 0.0
        for s in shippers:
            rew, _ = self._solve_routing(s, base_bag_assignments[s.id], t, T)
            base_bag_reward += rew
            
        best_assignment = base_bag_assignments
        best_reward = base_bag_reward
        
        if unpicked:
            # ACO Loop
            for _ in range(self.n_iterations):
                for _ in range(self.n_ants):
                    # Khởi tạo assignment cho kiến này bằng các đơn trong bag
                    ant_assigned_orders = {s.id: list(base_bag_assignments[s.id]) for s in shippers}
                    available = set(o.id for o in unpicked)
                    
                    # Random thứ tự shipper để tránh thiên vị
                    shuffled_shippers = list(shippers)
                    random.shuffle(shuffled_shippers)
                    
                    for s in shuffled_shippers:
                        base_rew, route = self._solve_routing(s, ant_assigned_orders[s.id], t, T)
                        # Tăng giới hạn gán đơn lên tối đa s.K_max + 3
                        while len(ant_assigned_orders[s.id]) < s.K_max + 3 and available:
                            choices = []
                            weights = []
                            rewards = {}
                            
                            # Pruning: Chỉ xem xét top 5 đơn hàng gần nhất để tránh bùng nổ tổ hợp
                            curr_s_pos = s.position
                            if route:
                                last_action, last_o = route[-1]
                                curr_s_pos = (last_o.ex, last_o.ey) if last_action == 'deliver' else (last_o.sx, last_o.sy)
                            
                            sorted_available = sorted(available, key=lambda oid: self._distance(curr_s_pos, (orders[oid].sx, orders[oid].sy)))
                            candidates_to_try = sorted_available[:5]
                            
                            for oid in candidates_to_try:
                                o = orders[oid]
                                current_weight = sum(x.w for x in ant_assigned_orders[s.id])
                                if current_weight + o.w > s.W_max:
                                    continue
                                    
                                new_rew, _ = self._solve_routing(s, ant_assigned_orders[s.id] + [o], t, T)
                                gain = new_rew - base_rew
                                if gain <= 0.001:
                                    continue
                                    
                                tau = self.pheromone.get((s.id, o.id), self.tau0)
                                prob = (tau ** self.alpha_aco) * (gain ** self.beta_aco)
                                choices.append(o)
                                weights.append(prob)
                                rewards[o.id] = new_rew
                                
                            if not choices:
                                break
                                
                            total_weight = sum(weights)
                            if total_weight <= 0.0:
                                chosen_o = random.choice(choices)
                            else:
                                chosen_o = random.choices(choices, weights=weights, k=1)[0]
                            ant_assigned_orders[s.id].append(chosen_o)
                            available.remove(chosen_o.id)
                            base_rew = rewards[chosen_o.id]
                            route = self._routing_cache[(s.id, tuple(sorted(o.id for o in ant_assigned_orders[s.id])))] [1]
                            
                    # Tính tổng reward của kiến này
                    ant_total_reward = 0.0
                    for s in shippers:
                        rew, _ = self._solve_routing(s, ant_assigned_orders[s.id], t, T)
                        ant_total_reward += rew
                        
                    if ant_total_reward > best_reward:
                        best_reward = ant_total_reward
                        best_assignment = ant_assigned_orders
                        
                # Evaporate pheromones
                for k in list(self.pheromone.keys()):
                    self.pheromone[k] *= (1.0 - self.rho)
                    
                # Deposit pheromones for the best assignment found
                if best_assignment:
                    for sid, assigned_list in best_assignment.items():
                        for o in assigned_list:
                            if o not in base_bag_assignments[sid]:
                                k = (sid, o.id)
                                self.pheromone[k] = self.pheromone.get(k, self.tau0) + (best_reward / max(len(unpicked), 1)) * 0.1

        # Xác định goal và operation ngay lập tức tiếp theo cho mỗi shipper
        goals: Dict[int, Position] = {}
        target_ops: Dict[int, int] = {}
        targeted_orders: Dict[int, Order] = {}
        
        for s in shippers:
            _, route = self._solve_routing(s, best_assignment.get(s.id, []), t, T)
            if route:
                action_type, o = route[0]
                targeted_orders[s.id] = o
                if action_type == 'pickup':
                    goals[s.id] = (o.sx, o.sy)
                    target_ops[s.id] = 1
                else:
                    goals[s.id] = (o.ex, o.ey)
                    target_ops[s.id] = 2
            else:
                # Nếu shipper hoàn toàn rảnh rỗi (idle), di chuyển thông minh
                assigned_oids = set()
                for other_s in shippers:
                    for o in best_assignment.get(other_s.id, []):
                        if not o.picked:
                            assigned_oids.add(o.id)
                rem_unassigned = [o for o in orders.values() if not o.picked and not o.delivered and o.id not in assigned_oids]
                if rem_unassigned:
                    nearest_o = min(rem_unassigned, key=lambda o: self._distance(s.position, (o.sx, o.sy)))
                    goals[s.id] = (nearest_o.sx, nearest_o.sy)
                    targeted_orders[s.id] = nearest_o
                else:
                    if self.proactive_idle:
                        goals[s.id] = self.centroid
                    else:
                        goals[s.id] = s.position
                target_ops[s.id] = 0
                
        # 2. Multi-Agent Path Finding với Cooperative A*
        starts = {s.id: s.position for s in shippers}
        solution_paths = self._cooperative_astar(starts, goals, shippers, targeted_orders)
        
        # 3. Chuyển thành _plans
        for s in shippers:
            path = solution_paths.get(s.id, [s.position])
            if len(path) > 1: path = path[1:]
            
            op = target_ops.get(s.id, 0)
            goal = goals.get(s.id, s.position)
            
            self._plans[s.id] = []
            for p in path:
                if p == goal:
                    self._plans[s.id].append((p, op))
                else:
                    self._plans[s.id].append((p, 0))

    def _decide_actions(self, obs: dict) -> Dict[int, Action]:
        if self._should_replan(obs):
            self._replan_aco(obs)
            
        actions: Dict[int, Action] = {}
        for shipper in obs["shippers"]:
            plan = self._plans.get(shipper.id, [])
            if not plan:
                actions[shipper.id] = ("S", 0)
                continue
                
            next_pos, target_op = plan[0]
            
            if shipper.position == next_pos:
                actions[shipper.id] = ("S", target_op)
                self._plans[shipper.id].pop(0)
            else:
                move = "S"
                for m in ("U", "D", "L", "R"):
                    if valid_next_pos(shipper.position, m, self.grid) == next_pos:
                        move = m
                        break
                actions[shipper.id] = (move, target_op)
                self._plans[shipper.id].pop(0)
                
        return actions

    def run(self) -> dict:
        start_time = time.time()
        obs = self.env.reset()

        while not obs.get("done", False):
            actions = self._decide_actions(obs)
            obs, _, done, _ = self.env.step(actions)
            if done: break

        return self.env.result(self.method_name, elapsed_sec=time.time() - start_time)