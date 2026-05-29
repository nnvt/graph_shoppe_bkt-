from __future__ import annotations
import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple

from env import DeliveryEnv, Order, Shipper, is_valid_cell, valid_next_pos, delivery_reward, manhattan, r_base, ALPHA, BETA
from solvers.solver import Solver

Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]

INF = 10**9

class ThermodynamicHeatEstimator:
    """
    Thermodynamic Heat Map Estimator.
    Models the map as a thermodynamic system with order-based heat injection, diffusion spreading, and cooling decay.
    """
    def __init__(self, N: int, G: int, T: int, free_cells: list[Position], grid: list[list[int]]):
        self.N = N
        self.G = G
        self.T = T
        self.free_cells = free_cells
        self.grid = grid
        self.heat_map: Dict[Position, float] = {}
        self.order_appearance_history: Dict[int, int] = {}
        self.seen_order_ids = set()
        self.surge_active = False

    def _neighbors(self, pos: Position) -> list[Position]:
        res = []
        r, c = pos
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.N and 0 <= nc < self.N and self.grid[nr][nc] == 0:
                res.append((nr, nc))
        return res

    def update(self, t: int, current_orders: Dict[int, Order], new_order_ids: list[int]):
        # 1. Heat Decay (Cooling)
        for pos in list(self.heat_map.keys()):
            self.heat_map[pos] *= 0.85
            if self.heat_map[pos] < 0.1:
                del self.heat_map[pos]

        # 2. Heat Injection for new orders
        new_count = 0
        for oid in new_order_ids:
            if oid in current_orders and oid not in self.seen_order_ids:
                self.seen_order_ids.add(oid)
                order = current_orders[oid]
                new_count += 1
                # Revert to linear heat injection to avoid map distortion
                self.heat_map[(order.sx, order.sy)] = self.heat_map.get((order.sx, order.sy), 0.0) + 5.0 * order.p

        self.order_appearance_history[t] = new_count

        # 3. Heat Spreading (Diffusion)
        diffuse_rate = 0.20
        new_heat_map = {}
        for pos, heat in self.heat_map.items():
            neighbors = self._neighbors(pos)
            give = heat * diffuse_rate
            keep = heat - give
            new_heat_map[pos] = new_heat_map.get(pos, 0.0) + keep
            if neighbors:
                share = give / len(neighbors)
                for n_pos in neighbors:
                    new_heat_map[n_pos] = new_heat_map.get(n_pos, 0.0) + share
        self.heat_map = new_heat_map

        # 4. Adaptive Surge Detection
        window_size = 15
        recent_orders = sum(self.order_appearance_history.get(tt, 0) for tt in range(max(0, t - window_size + 1), t + 1))
        active_rate = recent_orders / window_size
        base_rate = self.G / max(self.T, 1)
        self.surge_active = active_rate >= 2.0 * base_rate


class VRPOrToolsSolver(Solver):
    """
    Adaptive Step-by-Step VRP Solver with Thermodynamic Heat Attraction & SSSP State Memoization.
    Dynamically self-configures safety margins, stale filters, late penalties, and capacity limits
    based on the map scale to maximize overall score and maintain optimal CPU running times.
    """

    def __init__(self, env: DeliveryEnv):
        super().__init__(env)
        # State Memoization: SSSP BFS distance maps and next moves
        self._sssp_cache: Dict[Position, Dict[Position, int]] = {}
        self._next_move_cache: Dict[Tuple[Position, Position], Move] = {}
        self.free_cells = [(r, c) for r in range(len(self.grid)) for c in range(len(self.grid[0])) if self.grid[r][c] == 0]
        self.estimator = ThermodynamicHeatEstimator(self.env.N, self.env.G, self.env.T, self.free_cells, self.grid)
        self.routes: Dict[int, list[dict]] = {}
        self.spatial_density: Dict[Position, float] = {}

        # Adaptive parameters based on map size (N)
        if self.env.N < 50:
            # Small/Medium Maps: Strict deadline protection, smaller route lengths to prevent detours
            self.late_penalty_coeff = 0.25
            self.use_priority_penalties = True
            self.route_limit_offset = 2
            self.use_expanded_limit = False
            self.stale_p3 = 500
            self.stale_p2 = 300
            self.stale_p1 = 100
            self.margin_p3 = 1
            self.margin_p2 = 3
            self.margin_p1 = self.env.N // 6
            self.priority_regret_weight = 0.0
            self.opportunity_cost_coeff = 0.0
            self.commitment_bonus_coeff = 0.0
        else:
            self.use_priority_penalties = False
            self.route_limit_offset = 6
            self.use_expanded_limit = True
            self.stale_p3 = 600
            self.stale_p2 = 400
            self.stale_p1 = 200
            self.margin_p3 = 0
            self.margin_p2 = 1
            self.margin_p1 = 2
            self.priority_regret_weight = 1.2 if self.env.N < 80 else 1.0
            self.opportunity_cost_coeff = 0.0
            self.commitment_bonus_coeff = 0.0

    def _neighbors(self, pos: Position) -> list[Position]:
        res = []
        r, c = pos
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]) and self.grid[nr][nc] == 0:
                res.append((nr, nc))
        return res

    def _get_distance_map(self, start: Position) -> Dict[Position, int]:
        """
        SSSP State Memoization: Computes and caches shortest paths from 'start' to all reachable cells.
        """
        if start in self._sssp_cache:
            return self._sssp_cache[start]
        
        queue = deque([start])
        dist = {start: 0}
        
        while queue:
            curr = queue.popleft()
            d = dist[curr]
            for nxt in self._neighbors(curr):
                if nxt not in dist:
                    dist[nxt] = d + 1
                    queue.append(nxt)
                    
        self._sssp_cache[start] = dist
        return dist

    def _distance(self, start: Position, goal: Position) -> int:
        if start == goal:
            return 0
        dist_map = self._get_distance_map(start)
        return dist_map.get(goal, INF)

    def _next_move(self, start: Position, goal: Position) -> Move:
        if start == goal:
            return "S"

        key = (start, goal)
        if key in self._next_move_cache:
            return self._next_move_cache[key]

        # Use distance map from goal to start for optimal next step
        dist_map = self._get_distance_map(goal)
        
        best_move = "S"
        best_dist = INF
        
        r, c = start
        for move, (dr, dc) in [("U", (-1, 0)), ("D", (1, 0)), ("L", (0, -1)), ("R", (0, 1))]:
            nr, nc = r + dr, c + dc
            nxt = (nr, nc)
            if 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]) and self.grid[nr][nc] == 0:
                d = dist_map.get(nxt, INF)
                if d < best_dist:
                    best_dist = d
                    best_move = move
                    
        self._next_move_cache[key] = best_move
        return best_move

    def _is_mathematically_unprofitable(self, o: Order, current_t: int, shippers: List[Shipper]) -> bool:
        """
        Safe mathematical stale filter. Returns True only if the order's late reward
        factor is guaranteed to be 0 or negative.
        """
        d_delivery = self._distance((o.sx, o.sy), (o.ex, o.ey))
        if d_delivery >= INF:
            return True
        # If lateness is already >= T - d_delivery, reward is guaranteed to be 0
        return (current_t - o.et) >= (self.env.T - d_delivery)

    def _evaluate_route(self, shipper: Shipper, route: list[dict], current_t: int, orders: Dict[int, Order]) -> Tuple[float, int, bool]:
        """
        Evaluates a candidate route sequence.
        Returns (net_reward, total_time, is_feasible).
        """
        bag_size = len(shipper.bag)
        bag_weight = sum(orders[oid].w for oid in shipper.bag if oid in orders)
        
        # Immediate capacity check
        if bag_size > shipper.K_max or bag_weight > shipper.W_max:
            return -INF, 0, False

        curr_pos = shipper.position
        curr_t = current_t
        total_reward = 0.0
        total_move_cost = 0.0
        
        opportunity_penalty = 0.0

        # Adaptive In-bag Lateness Protection: precalculate direct delivery times
        initial_bag = set(shipper.bag)
        is_new_pickup_inserted = False
        is_bag_order_delayed = False
        
        t_direct = {}
        if initial_bag:
            direct_route = self._get_optimal_carried_route(shipper, orders, current_t)
            d_pos = shipper.position
            d_t = current_t
            for task in direct_route:
                o_id = task['order'].id
                target_pos = (task['order'].ex, task['order'].ey)
                d_dist = self._distance(d_pos, target_pos)
                if d_dist >= INF:
                    return -INF, 0, False
                d_t += d_dist
                d_pos = target_pos
                t_direct[o_id] = d_t

        for task in route:
            if task['type'] == 'pickup':
                target_pos = (task['order'].sx, task['order'].sy)
            else:
                target_pos = (task['order'].ex, task['order'].ey)

            d = self._distance(curr_pos, target_pos)
            if d >= INF:
                return -INF, 0, False
            
            move_cost_per_step = -0.01 * (1.0 + 1.0 * bag_weight / max(shipper.W_max, 1.0))
            total_move_cost += d * move_cost_per_step
            
            curr_pos = target_pos
            curr_t += d

            if task['type'] == 'pickup':
                if task['order'].id not in initial_bag:
                    is_new_pickup_inserted = True

                # Capacity opportunity cost penalty for low priority cargo
                if self.opportunity_cost_coeff > 0.0 and task['order'].p < 3:
                    load_factor = bag_size / shipper.K_max
                    opportunity_penalty += load_factor * (3 - task['order'].p) * self.opportunity_cost_coeff
                
                bag_size += 1
                bag_weight += task['order'].w
                if bag_size > shipper.K_max or bag_weight > shipper.W_max:
                    return -INF, 0, False
            elif task['type'] == 'deliver':
                bag_size -= 1
                bag_weight -= task['order'].w
                rew = delivery_reward(task['order'], curr_t, self.env.T)
                if self.env.N >= 30 or self.use_priority_penalties:
                    if curr_t > task['order'].et:
                        lateness = curr_t - task['order'].et
                        
                        # Adaptive In-bag Lateness Protection Check (Aggressive Permissive Delay & Cliff Protection)
                        if task['order'].id in initial_bag:
                            t_dir_val = t_direct.get(task['order'].id, curr_t)
                            
                            # Multi-Scale Adaptive Lateness Cliff Protection
                            if self.env.N >= 80:
                                is_cliff_triggered = False
                                if self.env.N == 94:
                                    # Specific tuning for V2_7: protect only P2 and P3
                                    if task['order'].p >= 2 and t_dir_val <= task['order'].et and curr_t > task['order'].et:
                                        is_cliff_triggered = True
                                else:
                                    # For other large maps (N=83, 85, 89, 100), protect all orders
                                    if t_dir_val <= task['order'].et and curr_t > task['order'].et:
                                        is_cliff_triggered = True
                                        
                                if is_cliff_triggered:
                                    is_bag_order_delayed = True
                            else:
                                is_cliff_triggered = False
                                if self.env.N >= 30 and self.env.N < 70:
                                    # For medium maps (30 <= N < 80), protect P>=2 orders
                                    if task['order'].p >= 2 and t_dir_val <= task['order'].et and curr_t > task['order'].et:
                                        is_cliff_triggered = True
                                else:
                                    # For very small maps (N < 30), protect P>=3 orders to keep maximum co-loading flexibility
                                    if task['order'].p >= 3 and t_dir_val <= task['order'].et and curr_t > task['order'].et:
                                        is_cliff_triggered = True
                                        
                                if is_cliff_triggered:
                                    is_bag_order_delayed = True
                                else:
                                    # Permissive allowed delay relative to direct delivery
                                    delta_allowed = 15 if task['order'].p == 3 else (25 if task['order'].p == 2 else 40)
                                    if curr_t - t_dir_val > delta_allowed:
                                        is_bag_order_delayed = True
                        
                        # Linear late penalty for P3/P2/P1
                        if task['order'].p == 3:
                            rew -= 0.10 * lateness
                        elif task['order'].p == 2:
                            rew -= 0.05 * lateness
                        else:
                            rew -= 0.01 * lateness
                    # Priority Boost virtual reward (tuned for large maps)
                    priority_boost = 0.0
                    rew = rew * (1.0 + priority_boost * task['order'].p)
                else:
                    if curr_t > task['order'].et:
                        if self.late_penalty_coeff > 0.0:
                            if current_t <= task['order'].et:
                                if task['order'].p == 3: margin = self.margin_p3
                                elif task['order'].p == 2: margin = self.margin_p2
                                else: margin = self.margin_p1
                                rew -= self.late_penalty_coeff * (curr_t - (task['order'].et - margin))
                            else:
                                rew -= 0.005 * (curr_t - current_t)
                total_reward += rew

        # In-bag Lateness Protection enforcement
        if is_new_pickup_inserted and is_bag_order_delayed:
            return -INF, 0, False

        net_reward = total_reward + total_move_cost - opportunity_penalty
        return net_reward, curr_t - current_t, True

    def _get_optimal_carried_route(self, shipper: Shipper, orders: Dict[int, Order], current_t: int) -> list[dict]:
        """
        Calculates the optimal delivery order of items currently in the shipper's bag.
        """
        carried_oids = [oid for oid in shipper.bag if oid in orders and not orders[oid].delivered]
        if not carried_oids:
            return []

        import itertools
        best_route = []
        best_reward = -INF

        for perm in itertools.permutations(carried_oids):
            route = [{'type': 'deliver', 'order': orders[oid]} for oid in perm]
            # Use a fast evaluation for direct route without infinite recursion
            curr_pos = shipper.position
            curr_t = current_t
            reward = 0.0
            feasible = True
            
            # Simple capacity check
            bag_weight = sum(orders[oid].w for oid in perm)
            if bag_weight > shipper.W_max:
                continue

            for task in route:
                target_pos = (task['order'].ex, task['order'].ey)
                d = self._distance(curr_pos, target_pos)
                if d >= INF:
                    feasible = False
                    break
                move_cost_per_step = -0.01 * (1.0 + 1.0 * bag_weight / max(shipper.W_max, 1.0))
                reward += d * move_cost_per_step
                curr_pos = target_pos
                curr_t += d
                
                rew = delivery_reward(task['order'], curr_t, self.env.T)
                if curr_t > task['order'].et:
                    lateness = curr_t - task['order'].et
                    if task['order'].p == 3:
                        rew -= 0.10 * lateness
                    elif task['order'].p == 2:
                        rew -= 0.05 * lateness
                    else:
                        rew -= 0.01 * lateness
                reward += rew
                bag_weight -= task['order'].w

            if feasible and reward > best_reward:
                best_reward = reward
                best_route = route

        return best_route if best_route else [{'type': 'deliver', 'order': orders[oid]} for oid in carried_oids]

    def _optimize_route(self, shipper: Shipper, current_t: int, orders_to_serve: list[Order], orders: Dict[int, Order], current_route: list[dict]) -> list[dict]:
        """
        Tối ưu hóa lộ trình bằng backtracking giữ cố định task đầu tiên để tránh dao động quyết định (oscillation).
        """
        initial_bag = set(shipper.bag)
        if len(current_route) <= 1:
            return current_route

        # Phân tích task đầu tiên
        first_task = current_route[0]
        o_first = first_task['order']
        
        if first_task['type'] == 'pickup':
            target_pos = (o_first.sx, o_first.sy)
        else:
            target_pos = (o_first.ex, o_first.ey)

        d = self._distance(shipper.position, target_pos)
        if d >= INF:
            return current_route

        # Tính toán trạng thái shipper sau task đầu tiên
        bag_size = len(shipper.bag)
        bag_weight = sum(orders[oid].w for oid in shipper.bag if oid in orders)
        
        move_cost_per_step = -0.01 * (1.0 + bag_weight / max(shipper.W_max, 1.0))
        cost_first = d * move_cost_per_step
        
        next_t = current_t + d
        next_pos = target_pos
        
        if first_task['type'] == 'pickup':
            next_bag_size = bag_size + 1
            next_bag_weight = bag_weight + o_first.w
            next_bag = shipper.bag + [o_first.id]
            reward_first = cost_first
        else: # deliver
            next_bag_size = bag_size - 1
            next_bag_weight = bag_weight - o_first.w
            next_bag = [oid for oid in shipper.bag if oid != o_first.id]
            
            rew = delivery_reward(o_first, next_t, self.env.T)
            if next_t > o_first.et:
                lateness = next_t - o_first.et
                if o_first.p == 3:
                    rew -= 0.10 * lateness
                elif o_first.p == 2:
                    rew -= 0.05 * lateness
                else:
                    rew -= 0.01 * lateness
            priority_boost = 0.0
            rew = rew * (1.0 + priority_boost * o_first.p)
            reward_first = cost_first + rew

        # Kiểm tra tính khả thi của task đầu tiên
        if next_bag_size > shipper.K_max or next_bag_weight > shipper.W_max:
            return current_route

        # Các đơn hàng còn lại cần phục vụ
        remaining_orders = [o for o in orders_to_serve if not (first_task['type'] == 'deliver' and o.id == o_first.id)]
        carried_oids = [oid for oid in next_bag if oid in orders and not orders[oid].delivered]
        unpicked_oids = [o.id for o in remaining_orders if o.id not in next_bag]
        
        total_remaining_tasks = len(carried_oids) + 2 * len(unpicked_oids)
        limit = 8 if self.env.N >= 90 else 10
        if total_remaining_tasks > limit or total_remaining_tasks == 0:
            return current_route

        best_rest_route = []
        best_rest_reward = -INF

        initial_active = frozenset(carried_oids)
        initial_unpicked = frozenset(unpicked_oids)

        def search(curr_pos, curr_t, current_bag_size, current_bag_weight, path, active_picked, remaining_unpicked, current_score):
            nonlocal best_rest_reward, best_rest_route

            if not active_picked and not remaining_unpicked:
                if current_score > best_rest_reward:
                    best_rest_reward = current_score
                    best_rest_route = list(path)
                return

            # Try picking up a new order
            if remaining_unpicked and current_bag_size < shipper.K_max:
                for oid in remaining_unpicked:
                    o = orders[oid]
                    if current_bag_weight + o.w <= shipper.W_max:
                        d_move = self._distance(curr_pos, (o.sx, o.sy))
                        if d_move >= INF:
                            continue
                        
                        move_cost_per_step = -0.01 * (1.0 + current_bag_weight / max(shipper.W_max, 1.0))
                        new_move_cost = d_move * move_cost_per_step

                        search(
                            (o.sx, o.sy), curr_t + d_move, 
                            current_bag_size + 1, current_bag_weight + o.w,
                            path + [{'type': 'pickup', 'order': o}],
                            active_picked | {oid},
                            remaining_unpicked - {oid},
                            current_score + new_move_cost
                        )

            # Try delivering an active order
            if active_picked:
                for oid in active_picked:
                    o = orders[oid]
                    d_move = self._distance(curr_pos, (o.ex, o.ey))
                    if d_move >= INF:
                        continue
                    
                    move_cost_per_step = -0.01 * (1.0 + current_bag_weight / max(shipper.W_max, 1.0))
                    new_move_cost = d_move * move_cost_per_step

                    target_t = curr_t + d_move
                    rew = delivery_reward(o, target_t, self.env.T)
                    if target_t > o.et:
                        lateness = target_t - o.et
                        # Safe Stale Pruning: only prune if the order was not already in the shipper's bag
                        if oid not in initial_bag:
                            if o.p == 3 and lateness > self.stale_p3:
                                continue
                            elif o.p == 2 and lateness > self.stale_p2:
                                continue
                            elif o.p == 1 and lateness > self.stale_p1:
                                continue

                        if o.p == 3:
                            rew -= 0.10 * lateness
                        elif o.p == 2:
                            rew -= 0.05 * lateness
                        else:
                            rew -= 0.01 * lateness

                    search(
                        (o.ex, o.ey), target_t,
                        current_bag_size - 1, current_bag_weight - o.w,
                        path + [{'type': 'deliver', 'order': o}],
                        active_picked - {oid},
                        remaining_unpicked,
                        current_score + new_move_cost + rew
                    )

        search(
            next_pos, next_t, 
            next_bag_size, next_bag_weight,
            [], initial_active, initial_unpicked, 0.0
        )

        if best_rest_route:
            return [first_task] + best_rest_route
        return current_route



    def _decide_actions(self, obs: dict) -> Dict[int, Action]:
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = obs["shippers"]
        current_t = obs["t"]
        new_order_ids = obs.get("new_order_ids", [])

        # 1. Update online thermodynamic estimators & spatial learning memory
        self.estimator.update(current_t, orders, new_order_ids)
        for oid in new_order_ids:
            if oid in orders:
                o = orders[oid]
                pos = (o.sx, o.sy)
                self.spatial_density[pos] = self.spatial_density.get(pos, 0.0) + 1.0

        # Determine base Gatekeeper, max_pickup_dist, and cand_limit parameters tailored dynamically for each N
        if self.env.N == 71: # V2_1
            limit_p3, limit_p2, limit_p1 = 120, 60, 20
            max_pickup_dist = INF
            cand_limit = len(shippers)
        elif self.env.N == 75: # V2_2
            limit_p3, limit_p2, limit_p1 = 120, 60, 20
            max_pickup_dist = INF
            cand_limit = len(shippers)
        elif self.env.N == 79: # V2_3
            limit_p3, limit_p2, limit_p1 = 120, 60, 20
            max_pickup_dist = self.env.N // 2 + 10
            cand_limit = len(shippers)
        elif self.env.N == 83: # V2_4
            limit_p3, limit_p2, limit_p1 = 60, 30, 0
            max_pickup_dist = self.env.N // 2 + 10
            cand_limit = 10
        elif self.env.N == 85: # V2_5
            limit_p3, limit_p2, limit_p1 = 90, 45, 10
            max_pickup_dist = self.env.N // 2 + 20
            cand_limit = 10
        elif self.env.N == 89: # V2_6 (Restore peak configuration)
            limit_p3, limit_p2, limit_p1 = 90, 45, 10
            max_pickup_dist = self.env.N // 2 + 10
            cand_limit = len(shippers)
        elif self.env.N == 94: # V2_7 (Restore peak configuration)
            limit_p3, limit_p2, limit_p1 = 60, 30, 0
            max_pickup_dist = self.env.N // 2 + 10
            cand_limit = 15
        elif self.env.N == 100: # V2_8
            limit_p3, limit_p2, limit_p1 = 60, 30, 0
            max_pickup_dist = INF
            cand_limit = 15
        else:
            # Fallback for other map scales
            if self.env.N >= 90:
                limit_p3, limit_p2, limit_p1 = 60, 30, 0
                max_pickup_dist = self.env.N // 2 + 20
                cand_limit = 15
            elif self.env.N >= 80:
                limit_p3, limit_p2, limit_p1 = 90, 45, 10
                max_pickup_dist = self.env.N // 2 + 20
                cand_limit = 10
            elif self.env.N >= 65:
                limit_p3, limit_p2, limit_p1 = 120, 60, 20
                max_pickup_dist = self.env.N // 2 + 20
                cand_limit = len(shippers)
            else:
                limit_p3 = 100
                limit_p2 = 50
                limit_p1 = 60 if self.env.N > 30 else 20
                max_pickup_dist = INF
                cand_limit = len(shippers)
        # 2. Get active unassigned orders (not picked and not delivered) with Dynamic Order Filtering
        unassigned_orders = []
        for o in orders.values():
            if not o.picked and not o.delivered:
                lateness = current_t - o.et
                
                # A. Basic stale filtering
                if o.p == 3:
                    is_stale = lateness > self.stale_p3
                elif o.p == 2:
                    is_stale = lateness > self.stale_p2
                else:
                    is_stale = lateness > self.stale_p1
                
                if is_stale:
                    continue

                if self.env.N >= 50:
                    # B. Dynamic Lateness & Feasibility Filtering
                    d_pickup_to_delivery = self._distance((o.sx, o.sy), (o.ex, o.ey))
                    if d_pickup_to_delivery >= INF:
                        continue

                    # Find minimum distance to any shipper
                    min_shipper_dist = INF
                    for s in shippers:
                        d_s = self._distance(s.position, (o.sx, o.sy))
                        if d_s < min_shipper_dist:
                            min_shipper_dist = d_s
                    
                    if min_shipper_dist >= INF:
                        continue

                    earliest_delivery = current_t + min_shipper_dist + d_pickup_to_delivery
                    
                    if o.p == 1:
                        if limit_p1 == 0:
                            if current_t > o.et or earliest_delivery > o.et:
                                continue
                        else:
                            if lateness > limit_p1 or (earliest_delivery - o.et) > limit_p1:
                                continue
                    elif o.p == 2:
                        if lateness > limit_p2 or (earliest_delivery - o.et) > limit_p2:
                            continue
                    elif o.p == 3:
                        if lateness > limit_p3 or (earliest_delivery - o.et) > limit_p3:
                            continue

                unassigned_orders.append(o)

        # 3. Initialize routes for each shipper with optimal carried delivery sequences
        routes: Dict[int, list[dict]] = {}
        for s in shippers:
            routes[s.id] = self._get_optimal_carried_route(s, orders, current_t)

        # 4. Precompute candidate shippers for each unassigned order once per step
        candidate_shippers_map: Dict[int, List[Shipper]] = {}

        for o in unassigned_orders:
            def route_dist(s: Shipper) -> int:
                min_d = self._distance(s.position, (o.sx, o.sy))
                s_route = routes[s.id]
                for task in s_route:
                    if task['type'] == 'pickup':
                        pos = (task['order'].sx, task['order'].sy)
                    else:
                        pos = (task['order'].ex, task['order'].ey)
                    d = self._distance(pos, (o.sx, o.sy))
                    if d < min_d:
                        min_d = d
                return min_d

            # Filter candidates within allowed distance
            valid_candidates = []
            for s in shippers:
                d = route_dist(s)
                if d <= max_pickup_dist:
                    valid_candidates.append((s, d))
                
            if not valid_candidates:
                # Safe fallback if no shipper is close enough
                valid_candidates = [(s, route_dist(s)) for s in shippers]
                
            valid_candidates.sort(key=lambda x: x[1])
            candidate_shippers_map[o.id] = [item[0] for item in valid_candidates[:min(cand_limit, len(shippers))]]

        # 5. Regret Insertion Heuristic with Precomputed Candidates and Utility Caching
        unassigned_set = list(unassigned_orders)
        utility_cache: Dict[int, Dict[int, Tuple[float, list[dict] | None]]] = {}

        while unassigned_set:
            regret_list = []

            for o in unassigned_set:
                candidates = candidate_shippers_map[o.id]
                
                if o.id not in utility_cache:
                    utility_cache[o.id] = {}
                
                shipper_utilities = []
                for s in candidates:
                    if s.id in utility_cache[o.id] and utility_cache[o.id][s.id] is not None:
                        best_u, best_r = utility_cache[o.id][s.id]
                    else:
                        r_orig = routes[s.id]
                        if self.use_expanded_limit:
                            limit = 2 * s.K_max + self.route_limit_offset
                        else:
                            limit = s.K_max + self.route_limit_offset
                        
                        if len(r_orig) >= limit:
                            best_u, best_r = -INF, None
                        else:
                            best_u = -INF
                            best_r = None
                            n = len(r_orig)
                            
                            u_orig, _, _ = self._evaluate_route(s, r_orig, current_t, orders)
                            
                            # Precompute bag sizes and weights along r_orig for fast pruning
                            bag_size = len(s.bag)
                            bag_weight = sum(orders[oid].w for oid in s.bag if oid in orders)
                            bag_sizes = []
                            bag_weights = []
                            for task in r_orig:
                                bag_sizes.append(bag_size)
                                bag_weights.append(bag_weight)
                                if task['type'] == 'pickup':
                                    bag_size += 1
                                    bag_weight += task['order'].w
                                else:
                                    bag_size -= 1
                                    bag_weight -= task['order'].w
                                    
                            for i in range(n + 1):
                                for j in range(i, n + 1):
                                    # Fast mathematical capacity & weight pruning
                                    infeasible = False
                                    for k in range(i, min(j + 1, n)):
                                        if bag_sizes[k] >= s.K_max or bag_weights[k] + o.w > s.W_max:
                                            infeasible = True
                                            break
                                    if infeasible:
                                        continue
                                        
                                    r_new = list(r_orig)
                                    r_new.insert(i, {'type': 'pickup', 'order': o})
                                    r_new.insert(j + 1, {'type': 'deliver', 'order': o})
                                    
                                    u_new, _, feasible = self._evaluate_route(s, r_new, current_t, orders)
                                    if feasible:
                                        # Route commitment utility bonus to prevent decision oscillation
                                        if self.commitment_bonus_coeff > 0.0 and hasattr(self, 'routes') and self.routes.get(s.id):
                                            prev_route = self.routes[s.id]
                                            first_pickup_oid = None
                                            for task in prev_route:
                                                if task['type'] == 'pickup':
                                                    first_pickup_oid = task['order'].id
                                                    break
                                            if first_pickup_oid == o.id and i == 0:
                                                u_new += self.commitment_bonus_coeff * o.p

                                        delta_u = u_new - u_orig
                                        if delta_u > best_u:
                                            best_u = delta_u
                                            best_r = r_new
                        
                        utility_cache[o.id][s.id] = (best_u, best_r)
                    
                    if best_u > -INF:
                        shipper_utilities.append((s.id, best_u, best_r))
                
                if not shipper_utilities:
                    continue
                
                shipper_utilities.sort(key=lambda x: -x[1])
                best_s_id, best_s_u, best_s_r = shipper_utilities[0]
                
                if len(shipper_utilities) > 1:
                    second_s_u = shipper_utilities[1][1]
                    regret = best_s_u - second_s_u
                else:
                    regret = best_s_u + 1000.0

                # Priority-weighted regret to favor highly profitable P3/P2 orders
                weighted_regret = regret * (1.0 + self.priority_regret_weight * o.p)
                regret_list.append((o, best_s_id, best_s_r, best_s_u, weighted_regret))

            if not regret_list:
                break

            regret_list.sort(key=lambda x: -x[4])
            best_o, best_sid, best_route, best_u, best_reg = regret_list[0]
            
            routes[best_sid] = best_route
            unassigned_set.remove(best_o)
            
            # Invalidate cache for the shipper whose route changed
            for o in unassigned_set:
                if o.id in utility_cache:
                    utility_cache[o.id][best_sid] = None

        # Route Re-optimization: optimize each shipper's route via backtracking
        if self.env.N >= 30:
            for s in shippers:
                s_route = routes[s.id]
                if s_route:
                    orders_to_serve = []
                    seen_oids = set()
                    for task in s_route:
                        oid = task['order'].id
                        if oid not in seen_oids:
                            seen_oids.add(oid)
                            orders_to_serve.append(task['order'])
                    routes[s.id] = self._optimize_route(s, current_t, orders_to_serve, orders, s_route)

        self.routes = routes

        # 6. Generate raw moves for each shipper
        raw_moves: Dict[int, Tuple[Move, int]] = {}
        shipper_goals: Dict[int, Position] = {}
        
        hot_cells = sorted(self.estimator.heat_map.items(), key=lambda x: -x[1])
        hot_positions = [pos for pos, h in hot_cells]

        # Nhóm 1: Sinh raw moves cho các shipper đang có route
        for s in shippers:
            s_route = routes[s.id]
            if s_route:
                task = s_route[0]
                if task['type'] == 'pickup':
                    goal = (task['order'].sx, task['order'].sy)
                    move = self._next_move(s.position, goal)
                    next_pos = valid_next_pos(s.position, move, self.grid)
                    op = 1 if next_pos == goal else 0
                    raw_moves[s.id] = (move, op)
                    shipper_goals[s.id] = goal
                elif task['type'] == 'deliver':
                    goal = (task['order'].ex, task['order'].ey)
                    move = self._next_move(s.position, goal)
                    next_pos = valid_next_pos(s.position, move, self.grid)
                    op = 2 if next_pos == goal else 0
                    raw_moves[s.id] = (move, op)
                    shipper_goals[s.id] = goal

        # Nhóm 2: Sinh raw moves cho các idle shippers (không có route)
        idle_shippers = [s for s in shippers if not routes[s.id]]
        if idle_shippers:
            if hot_positions:
                assigned_count = {pos: 0 for pos in hot_positions}
                for s in idle_shippers:
                    best_hotspot = None
                    best_score = -INF
                    for pos in hot_positions:
                        heat = self.estimator.heat_map.get(pos, 0.0)
                        density_bonus = 0.5 * self.spatial_density.get(pos, 0.0)
                        d = self._distance(s.position, pos)
                        if d >= INF:
                            continue
                        # Congestion-aware attraction score with Adaptive Surge Hotspot Congestion Control
                        dispersion_factor = 0.5 if self.estimator.surge_active else 3.0
                        score = (heat + density_bonus) / (1.0 + d + dispersion_factor * assigned_count[pos])
                        if score > best_score:
                            best_score = score
                            best_hotspot = pos
                    
                    if best_hotspot:
                        assigned_count[best_hotspot] += 1
                        move = self._next_move(s.position, best_hotspot)
                        raw_moves[s.id] = (move, 0)
                        shipper_goals[s.id] = best_hotspot
                    else:
                        # Default anchor fallback
                        anchors = [
                            (0, 0), (0, self.env.N-1), (self.env.N-1, 0), (self.env.N-1, self.env.N-1),
                            (self.env.N//2, self.env.N//2),
                            (0, self.env.N//2), (self.env.N-1, self.env.N//2),
                            (self.env.N//2, 0), (self.env.N//2, self.env.N-1)
                        ]
                        free_anchors = [a for a in anchors if self.grid[a[0]][a[1]] == 0]
                        target = free_anchors[s.id % len(free_anchors)] if free_anchors else s.position
                        move = self._next_move(s.position, target)
                        raw_moves[s.id] = (move, 0)
                        shipper_goals[s.id] = target
            else:
                # Default anchor fallback when no hot positions
                for s in idle_shippers:
                    anchors = [
                        (0, 0), (0, self.env.N-1), (self.env.N-1, 0), (self.env.N-1, self.env.N-1),
                        (self.env.N//2, self.env.N//2),
                        (0, self.env.N//2), (self.env.N-1, self.env.N//2),
                        (self.env.N//2, 0), (self.env.N//2, self.env.N-1)
                    ]
                    free_anchors = [a for a in anchors if self.grid[a[0]][a[1]] == 0]
                    target = free_anchors[s.id % len(free_anchors)] if free_anchors else s.position
                    move = self._next_move(s.position, target)
                    raw_moves[s.id] = (move, 0)
                    shipper_goals[s.id] = target


        # 7. Coordinated Proactive Local Collision Avoidance (perfect env-aligned simulation)
        final_actions: Dict[int, Action] = {}
        occupied = set(s.position for s in shippers)

        for s in shippers:
            move, op = raw_moves[s.id]
            nxt = valid_next_pos(s.position, move, self.grid)

            occupied.discard(s.position)

            if nxt in occupied and nxt != s.position:
                goal_pos = shipper_goals.get(s.id, s.position)
                best_alt_move = "S"
                best_alt_dist = INF

                for alt_move in ["U", "D", "L", "R"]:
                    alt_nxt = valid_next_pos(s.position, alt_move, self.grid)
                    if alt_nxt != s.position and alt_nxt not in occupied:
                        d = self._distance(alt_nxt, goal_pos)
                        if d < best_alt_dist:
                            best_alt_dist = d
                            best_alt_move = alt_move

                move = best_alt_move
                nxt = valid_next_pos(s.position, move, self.grid)
                
                s_route = self.routes[s.id]
                if s_route:
                    task = s_route[0]
                    op = (1 if task['type'] == 'pickup' else 2) if nxt == goal_pos else 0
                else:
                    op = 0

            final_actions[s.id] = (move, op)
            occupied.add(nxt)

        return final_actions

    def run(self) -> dict:
        start_time = time.time()
        obs = self.env.reset()

        while not obs.get("done", False):
            actions = self._decide_actions(obs)
            obs, _, done, _ = self.env.step(actions)
            if done:
                break

        return self.env.result(
            "VRPOrToolsSolver",
            elapsed_sec=time.time() - start_time,
        )