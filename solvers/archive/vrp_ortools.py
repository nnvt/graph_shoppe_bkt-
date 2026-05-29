from __future__ import annotations

import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple

from env import DeliveryEnv, Order, Shipper, is_valid_cell, r_base, valid_next_pos
from solvers.solver import Solver

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except Exception:  # pragma: no cover - fallback for machines without OR-Tools
    pywrapcp = None
    routing_enums_pb2 = None


Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]

INF = 10**9
MOVES: Tuple[Move, ...] = ("U", "D", "L", "R")
SCALE_WEIGHT = 10


class VRPOrToolsSolver(Solver):
    """
    Rolling-horizon VRP baseline.

    At every timestep, the solver models currently visible pickup orders as an
    optional multi-vehicle routing problem. OR-Tools assigns promising pickups
    to idle shippers under count/weight capacity constraints; only the first
    move of the selected route is executed before replanning.
    """

    method_name = "VRP-OrTools"

    def __init__(self, env: DeliveryEnv):
        super().__init__(env)
        self._distance_cache: Dict[Tuple[Position, Position], int] = {}
        self._next_move_cache: Dict[Tuple[Position, Position], Move] = {}

    # ------------------------------------------------------------------
    # Grid shortest path helpers
    # ------------------------------------------------------------------
    def _neighbors(self, pos: Position) -> Iterable[Tuple[Move, Position]]:
        for move in MOVES:
            nxt = valid_next_pos(pos, move, self.grid)
            if nxt != pos:
                yield move, nxt

    def _bfs_parents(
        self,
        start: Position,
        goal: Position,
    ) -> Optional[Dict[Position, Tuple[Optional[Position], Move]]]:
        if not is_valid_cell(start, self.grid) or not is_valid_cell(goal, self.grid):
            return None

        queue: deque[Position] = deque([start])
        parent: Dict[Position, Tuple[Optional[Position], Move]] = {start: (None, "S")}

        while queue:
            current = queue.popleft()
            if current == goal:
                return parent
            for move, nxt in self._neighbors(current):
                if nxt in parent:
                    continue
                parent[nxt] = (current, move)
                queue.append(nxt)
        return None

    def _distance(self, start: Position, goal: Position) -> int:
        if start == goal:
            return 0
        key = (start, goal)
        if key in self._distance_cache:
            return self._distance_cache[key]

        parent = self._bfs_parents(start, goal)
        if parent is None or goal not in parent:
            self._distance_cache[key] = INF
            return INF

        dist = 0
        current = goal
        while current != start:
            previous, _ = parent[current]
            if previous is None:
                self._distance_cache[key] = INF
                return INF
            current = previous
            dist += 1
        self._distance_cache[key] = dist
        return dist

    def _next_move(self, start: Position, goal: Position) -> Move:
        if start == goal:
            return "S"
        key = (start, goal)
        if key in self._next_move_cache:
            return self._next_move_cache[key]

        parent = self._bfs_parents(start, goal)
        if parent is None or goal not in parent:
            self._next_move_cache[key] = "S"
            return "S"

        current = goal
        while True:
            previous, move = parent[current]
            if previous is None:
                self._next_move_cache[key] = "S"
                return "S"
            if previous == start:
                self._next_move_cache[key] = move
                return move
            current = previous

    # ------------------------------------------------------------------
    # Scoring and action helpers
    # ------------------------------------------------------------------
    def _reward_proxy(self, order: Order) -> float:
        return {1: 1.0, 2: 2.0, 3: 3.0}[order.p] * r_base(order.w)

    def _move_towards(self, shipper: Shipper, goal: Position) -> Tuple[Move, Position]:
        move = self._next_move(shipper.position, goal)
        return move, valid_next_pos(shipper.position, move, self.grid)

    def _delivery_order(self, shipper: Shipper, orders: Dict[int, Order]) -> Optional[Order]:
        carried = [
            orders[oid]
            for oid in shipper.bag
            if oid in orders and not orders[oid].delivered
        ]
        if not carried:
            return None
        return min(
            carried,
            key=lambda o: (
                max(0, o.et),
                self._distance(shipper.position, (o.ex, o.ey)),
                -o.p,
                o.id,
            ),
        )

    def _fallback_pickup(
        self,
        shipper: Shipper,
        orders: Dict[int, Order],
        reserved: set[int],
        t: int,
    ) -> Optional[Order]:
        best: Optional[Order] = None
        best_score = -10**18

        for order in orders.values():
            if order.id in reserved or not shipper.can_carry(order, orders):
                continue
            d1 = self._distance(shipper.position, (order.sx, order.sy))
            d2 = self._distance((order.sx, order.sy), (order.ex, order.ey))
            if d1 >= INF or d2 >= INF:
                continue
            eta = t + d1 + d2
            lateness = max(0, eta - order.et)
            urgency = max(0, order.et - t)
            score = (
                4.0 * self._reward_proxy(order)
                + 8.0 * order.p
                - 1.2 * d1
                - 0.4 * d2
                - 2.5 * lateness
                - 0.02 * urgency
            )
            if score > best_score or (score == best_score and order.id < (best.id if best else INF)):
                best_score = score
                best = order
        return best

    def _delivery_action(self, shipper: Shipper, order: Order) -> Action:
        goal = (order.ex, order.ey)
        move, nxt = self._move_towards(shipper, goal)
        return (move, 2) if nxt == goal else (move, 0)

    def _pickup_action(self, shipper: Shipper, order: Order) -> Action:
        goal = (order.sx, order.sy)
        move, nxt = self._move_towards(shipper, goal)
        return (move, 1) if nxt == goal else (move, 0)

    # ------------------------------------------------------------------
    # OR-Tools PDP routing model
    # ------------------------------------------------------------------
    def _candidate_orders(self, shippers: List[Shipper], orders: Dict[int, Order]) -> List[Order]:
        candidates: List[Order] = []
        for order in orders.values():
            if order.picked or order.delivered:
                continue
            if not any(s.can_carry(order, orders) for s in shippers):
                continue
            pickup = (order.sx, order.sy)
            delivery = (order.ex, order.ey)
            if self._distance(pickup, delivery) >= INF:
                continue
            candidates.append(order)

        # Sort by a score that combines priority, reward, and distance to shippers
        def order_score(o: Order) -> float:
            min_dist = min(self._distance(s.position, (o.sx, o.sy)) for s in shippers)
            return 30.0 * o.p + 2.0 * self._reward_proxy(o) - 1.5 * min_dist - 0.1 * o.et

        candidates.sort(key=lambda o: -order_score(o))
        return candidates[:12]

    def _solve_pickup_vrp(
        self,
        shippers: List[Shipper],
        orders: Dict[int, Order],
        t: int,
    ) -> Tuple[Dict[int, Tuple[Position, int]], set[int]]:
        if pywrapcp is None or not shippers:
            return {}, set()

        available_orders = self._candidate_orders(shippers, orders)
        
        # Build carried orders list
        carried_orders_list = []
        for v_idx, shipper in enumerate(shippers):
            for oid in shipper.bag:
                if oid in orders and not orders[oid].delivered:
                    carried_orders_list.append((v_idx, oid, orders[oid]))

        if not available_orders and not carried_orders_list:
            return {}, set()

        vehicle_count = len(shippers)
        num_available = len(available_orders)
        num_carried = len(carried_orders_list)
        
        # Nodes: starts + 2 * num_available + num_carried + ends
        total_nodes = vehicle_count + 2 * num_available + num_carried + vehicle_count
        starts = list(range(vehicle_count))
        ends = list(range(vehicle_count + 2 * num_available + num_carried, total_nodes))

        node_positions = []
        # Start nodes
        for s in shippers:
            node_positions.append(s.position)
        # Available pickups and deliveries
        for o in available_orders:
            node_positions.append((o.sx, o.sy))
            node_positions.append((o.ex, o.ey))
        # Carried deliveries
        for v_idx, oid, o in carried_orders_list:
            node_positions.append((o.ex, o.ey))
        # End nodes
        for s in shippers:
            node_positions.append(s.position)

        manager = pywrapcp.RoutingIndexManager(total_nodes, vehicle_count, starts, ends)
        routing = pywrapcp.RoutingModel(manager)

        # Distance callback and dimension
        def distance_cb(from_index: int, to_index: int) -> int:
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            dist = self._distance(node_positions[from_node], node_positions[to_node])
            return 100000 if dist >= INF else int(10 * dist)

        transit_idx = routing.RegisterTransitCallback(distance_cb)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)
        
        routing.AddDimension(
            transit_idx,
            0,
            2400,
            True,
            "Distance"
        )

        # Weight capacity dimension
        def weight_cb(from_index: int) -> int:
            node = manager.IndexToNode(from_index)
            # Available pickups
            if vehicle_count <= node < vehicle_count + 2 * num_available:
                offset = node - vehicle_count
                if offset % 2 == 0:  # pickup
                    return int(round(available_orders[offset // 2].w * SCALE_WEIGHT))
                else:  # delivery
                    return -int(round(available_orders[offset // 2].w * SCALE_WEIGHT))
            # Carried deliveries
            elif vehicle_count + 2 * num_available <= node < vehicle_count + 2 * num_available + num_carried:
                offset = node - (vehicle_count + 2 * num_available)
                return -int(round(carried_orders_list[offset][2].w * SCALE_WEIGHT))
            return 0

        weight_idx = routing.RegisterUnaryTransitCallback(weight_cb)
        routing.AddDimensionWithVehicleCapacity(
            weight_idx,
            10000,  # slack
            [int(round(s.W_max * SCALE_WEIGHT)) for s in shippers],
            False,  # fix_start_cumul_to_zero must be False because of initial loads!
            "Weight"
        )
        
        # Set initial loads for weight
        weight_dimension = routing.GetDimensionOrDie("Weight")
        for v in range(vehicle_count):
            start_index = routing.Start(v)
            initial_w = sum(orders[oid].w for oid in shippers[v].bag if oid in orders)
            weight_dimension.CumulVar(start_index).SetValue(int(round(initial_w * SCALE_WEIGHT)))

        # Count capacity dimension
        def count_cb(from_index: int) -> int:
            node = manager.IndexToNode(from_index)
            # Available pickups/deliveries
            if vehicle_count <= node < vehicle_count + 2 * num_available:
                offset = node - vehicle_count
                return 1 if offset % 2 == 0 else -1
            # Carried deliveries
            elif vehicle_count + 2 * num_available <= node < vehicle_count + 2 * num_available + num_carried:
                return -1
            return 0

        count_idx = routing.RegisterUnaryTransitCallback(count_cb)
        routing.AddDimensionWithVehicleCapacity(
            count_idx,
            100,  # slack
            [int(s.K_max) for s in shippers],
            False,  # fix_start_cumul_to_zero must be False because of initial loads!
            "Count"
        )

        # Set initial loads for count
        count_dimension = routing.GetDimensionOrDie("Count")
        for v in range(vehicle_count):
            start_index = routing.Start(v)
            count_dimension.CumulVar(start_index).SetValue(len(shippers[v].bag))

        # Add Pickup and Delivery relationships and Vehicle constraints
        for i in range(num_available):
            p_node = vehicle_count + 2 * i
            d_node = vehicle_count + 2 * i + 1
            p_idx = manager.NodeToIndex(p_node)
            d_idx = manager.NodeToIndex(d_node)
            routing.AddPickupAndDelivery(p_idx, d_idx)
            routing.solver().Add(routing.VehicleVar(p_idx) == routing.VehicleVar(d_idx))
            routing.solver().Add(
                routing.GetDimensionOrDie("Distance").CumulVar(p_idx)
                <= routing.GetDimensionOrDie("Distance").CumulVar(d_idx)
            )

        # Restrict carried deliveries to their vehicles
        for i, (v_idx, oid, o) in enumerate(carried_orders_list):
            d_node = vehicle_count + 2 * num_available + i
            d_idx = manager.NodeToIndex(d_node)
            routing.VehicleVar(d_idx).SetValues([v_idx])

        # Add disjunctions for available pickups to allow dropping them
        for i, order in enumerate(available_orders):
            p_node = vehicle_count + 2 * i
            p_idx = manager.NodeToIndex(p_node)
            # Find min distance to any vehicle to estimate ETA
            min_dist = min(self._distance(s.position, (order.sx, order.sy)) for s in shippers)
            eta = t + min_dist + self._distance((order.sx, order.sy), (order.ex, order.ey))
            lateness = max(0, eta - order.et)
            penalty = int(max(100, 80 * self._reward_proxy(order) + 150 * order.p - 40 * lateness))
            routing.AddDisjunction([p_idx], penalty)

        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        params.time_limit.FromMilliseconds(25)

        solution = routing.SolveWithParameters(params)
        if solution is None:
            return {}, set()

        targets = {}
        used_orders = set()
        for vehicle_idx, shipper in enumerate(shippers):
            index = routing.Start(vehicle_idx)
            next_index = solution.Value(routing.NextVar(index))
            if not routing.IsEnd(next_index):
                node = manager.IndexToNode(next_index)
                if vehicle_count <= node < vehicle_count + 2 * num_available:
                    offset = node - vehicle_count
                    order_idx = offset // 2
                    is_delivery = (offset % 2 == 1)
                    order = available_orders[order_idx]
                    if is_delivery:
                        targets[shipper.id] = ((order.ex, order.ey), 2)
                    else:
                        targets[shipper.id] = ((order.sx, order.sy), 1)
                        used_orders.add(order.id)
                elif vehicle_count + 2 * num_available <= node < vehicle_count + 2 * num_available + num_carried:
                    offset = node - (vehicle_count + 2 * num_available)
                    _, _, order = carried_orders_list[offset]
                    targets[shipper.id] = ((order.ex, order.ey), 2)

        return targets, used_orders

    # ------------------------------------------------------------------
    # Space-time CBS path planning
    # ------------------------------------------------------------------
    def _move_between(self, start: Position, nxt: Position) -> Move:
        if start == nxt:
            return "S"
        for move in MOVES:
            if valid_next_pos(start, move, self.grid) == nxt:
                return move
        return "S"

    def _violates_constraints(
        self,
        sid: int,
        current: Position,
        nxt: Position,
        next_t: int,
        vertex_constraints: set[Tuple[int, Position, int]],
        edge_constraints: set[Tuple[int, Position, Position, int]],
    ) -> bool:
        return (
            (sid, nxt, next_t) in vertex_constraints
            or (sid, current, nxt, next_t) in edge_constraints
        )

    def _space_time_path(
        self,
        sid: int,
        start: Position,
        goal: Position,
        horizon: int,
        vertex_constraints: set[Tuple[int, Position, int]],
        edge_constraints: set[Tuple[int, Position, Position, int]],
    ) -> List[Position]:
        if start == goal:
            return [start]

        ALL_MOVES = ("S", "U", "D", "L", "R")
        queue: deque[Tuple[Position, int]] = deque([(start, 0)])
        parent: Dict[Tuple[Position, int], Tuple[Optional[Tuple[Position, int]], Position]] = {
            (start, 0): (None, start)
        }
        best_state = (start, 0)
        best_dist = self._distance(start, goal)

        while queue:
            current, tau = queue.popleft()
            current_dist = self._distance(current, goal)
            if current_dist < best_dist:
                best_dist = current_dist
                best_state = (current, tau)

            if current == goal:
                best_state = (current, tau)
                break
            if tau >= horizon:
                continue

            for move in ALL_MOVES:
                nxt = valid_next_pos(current, move, self.grid)
                next_t = tau + 1
                if self._violates_constraints(sid, current, nxt, next_t, vertex_constraints, edge_constraints):
                    continue
                state = (nxt, next_t)
                if state in parent:
                    continue
                parent[state] = ((current, tau), nxt)
                queue.append(state)

        path_rev: List[Position] = []
        state: Optional[Tuple[Position, int]] = best_state
        while state is not None:
            path_rev.append(state[0])
            prev, _ = parent[state]
            state = prev
        path_rev.reverse()
        return path_rev or [start]

    def _path_at(self, path: List[Position], tau: int) -> Position:
        if not path:
            raise ValueError("empty path")
        return path[tau] if tau < len(path) else path[-1]

    def _find_conflict(self, paths: Dict[int, List[Position]], horizon: int) -> Optional[Tuple[str, int, int, Position, Position, int]]:
        sids = sorted(paths)
        for tau in range(1, horizon + 1):
            occupied: Dict[Position, int] = {}
            for sid in sids:
                pos = self._path_at(paths[sid], tau)
                other = occupied.get(pos)
                if other is not None:
                    return ("vertex", other, sid, pos, pos, tau)
                occupied[pos] = sid

            for i, sid_a in enumerate(sids):
                a_prev = self._path_at(paths[sid_a], tau - 1)
                a_now = self._path_at(paths[sid_a], tau)
                for sid_b in sids[i + 1:]:
                    b_prev = self._path_at(paths[sid_b], tau - 1)
                    b_now = self._path_at(paths[sid_b], tau)
                    if a_prev == b_now and b_prev == a_now and a_now != b_now:
                        return ("edge", sid_a, sid_b, b_prev, b_now, tau)
        return None

    def _cbs_paths(self, obs: dict, targets: Dict[int, Tuple[Position, int]]) -> Dict[int, List[Position]]:
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)
        shipper_by_id = {s.id: s for s in shippers}
        horizon = min(12, max(6, int(obs.get("N", 10)) // 2))
        vertex_constraints: set[Tuple[int, Position, int]] = set()
        edge_constraints: set[Tuple[int, Position, Position, int]] = set()

        paths: Dict[int, List[Position]] = {}
        for shipper in shippers:
            goal, _ = targets[shipper.id]
            paths[shipper.id] = self._space_time_path(
                shipper.id,
                shipper.position,
                goal,
                horizon,
                vertex_constraints,
                edge_constraints,
            )

        for _ in range(40):
            conflict = self._find_conflict(paths, horizon)
            if conflict is None:
                break

            kind, sid_a, sid_b, pos_a, pos_b, tau = conflict
            loser = max(sid_a, sid_b)
            if kind == "vertex":
                vertex_constraints.add((loser, pos_a, tau))
            else:
                edge_constraints.add((loser, pos_a, pos_b, tau))

            shipper = shipper_by_id[loser]
            goal, _ = targets[loser]
            paths[loser] = self._space_time_path(
                loser,
                shipper.position,
                goal,
                horizon,
                vertex_constraints,
                edge_constraints,
            )

        return paths

    def _assign_targets(self, obs: dict) -> Dict[int, Tuple[Position, int]]:
        t = int(obs["t"])
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)

        targets = {}
        used_orders = set()
        
        if pywrapcp is not None:
            targets, used_orders = self._solve_pickup_vrp(shippers, orders, t)

        # Fallback for shippers that don't have targets
        for shipper in shippers:
            if shipper.id not in targets:
                # 1. Best delivery if carrying packages
                delivery = self._delivery_order(shipper, orders)
                if delivery is not None:
                    targets[shipper.id] = ((delivery.ex, delivery.ey), 2)
                else:
                    # 2. Best pickup
                    pickup = self._fallback_pickup(shipper, orders, used_orders, t)
                    if pickup is not None:
                        targets[shipper.id] = ((pickup.sx, pickup.sy), 1)
                        used_orders.add(pickup.id)
                    else:
                        # 3. Idle
                        targets[shipper.id] = (shipper.position, 0)
        return targets

    def _decide_actions(self, obs: dict) -> Dict[int, Action]:
        targets = self._assign_targets(obs)
        paths = self._cbs_paths(obs, targets)
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)
        orders: Dict[int, Order] = obs["orders"]

        actions: Dict[int, Action] = {}
        for shipper in shippers:
            path = paths.get(shipper.id, [shipper.position])
            next_pos = path[1] if len(path) > 1 else path[0]
            move = self._move_between(shipper.position, next_pos)
            goal, op = targets[shipper.id]
            
            final_op = 0
            if next_pos == goal:
                final_op = op
                
            # Walk-over delivery: deliver if next_pos is destination of any carried order
            if any((orders[oid].ex, orders[oid].ey) == next_pos for oid in shipper.bag if oid in orders):
                final_op = 2
                
            actions[shipper.id] = (move, final_op)

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
