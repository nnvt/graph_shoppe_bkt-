from __future__ import annotations

import json
import heapq
import time
from collections import deque
from pathlib import Path as FilePath
from typing import Dict, Iterable, List, Optional, Tuple

from env import DeliveryEnv, Order, Shipper, is_valid_cell, r_base, valid_next_pos
from solvers.solver import Solver


Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]
Path = List[Position]

INF = 10**9
MOVES: Tuple[Move, ...] = ("U", "D", "L", "R")
WAIT_MOVES: Tuple[Move, ...] = ("S", "U", "D", "L", "R")
DEBUG_TRACE = True
TRACE_EVERY_LARGE = 25


class MAPDCBSSolver(Solver):
    """
    MAPD-CBS baseline.

    A greedy MAPD layer assigns each shipper one visible pickup/delivery target.
    A bounded CBS layer then replans paths in space-time to avoid vertex and
    edge conflicts, and the solver executes the first action only.
    """

    method_name = "MAPD-CBS"

    def __init__(self, env: DeliveryEnv):
        super().__init__(env)
        self._distance_cache: Dict[Tuple[Position, Position], int] = {}
        self._next_move_cache: Dict[Tuple[Position, Position], Move] = {}
        config_name = getattr(env, "config_name", "unknown")
        self._trace_path = FilePath(f"/tmp/solver_trace_{self.method_name}_{config_name}.jsonl")
        self._large_map_mode = self.env.N >= 40

    def _nearest_pickup_distance(self, shipper: Shipper, orders: Dict[int, Order]) -> int:
        distances = [
            self._bfs_distance(shipper.position, (order.sx, order.sy))
            for order in orders.values()
            if not order.picked and not order.delivered and shipper.can_carry(order, orders)
        ]
        return min(distances) if distances else INF

    def _trace_step(self, before: dict, actions: Dict[int, Action], after: dict) -> None:
        if not DEBUG_TRACE:
            return
        if self._large_map_mode and int(before["t"]) % TRACE_EVERY_LARGE != 0:
            return
        before_orders: Dict[int, Order] = before["orders"]
        after_orders: Dict[int, Order] = after["orders"]
        before_shippers: List[Shipper] = before["shippers"]
        after_by_id = {shipper.id: shipper for shipper in after["shippers"]}
        picked = [
            oid
            for oid, order in after_orders.items()
            if oid in before_orders and not before_orders[oid].picked and order.picked
        ]
        delivered = [
            oid
            for oid, order in before_orders.items()
            if order.picked and oid not in after_orders
        ]
        blocked_or_wait = []
        for shipper in before_shippers:
            move = actions.get(shipper.id, ("S", 0))[0]
            after_shipper = after_by_id.get(shipper.id)
            if after_shipper is None:
                continue
            if move == "S" or after_shipper.position == shipper.position and move != "S":
                blocked_or_wait.append(shipper.id)
        row = {
            "t": before["t"],
            "active_orders": len(before_orders),
            "new_orders": len(before.get("new_order_ids", [])),
            "picked_order_ids": picked,
            "delivered_order_ids": delivered,
            "idle_shippers": [s.id for s in before_shippers if not s.bag],
            "bag_sizes": {s.id: len(s.bag) for s in before_shippers},
            "planned_actions": {sid: list(action) for sid, action in actions.items()},
            "blocked_or_wait_actions": blocked_or_wait,
            "nearest_pickup_distance": {
                s.id: self._nearest_pickup_distance(s, before_orders) for s in before_shippers
            },
        }
        with self._trace_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------
    def _neighbors(self, pos: Position) -> Iterable[Tuple[Move, Position]]:
        for move in MOVES:
            nxt = valid_next_pos(pos, move, self.grid)
            if nxt != pos:
                yield move, nxt

    def _manhattan(self, start: Position, goal: Position) -> int:
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def _greedy_next_move(self, start: Position, goal: Position) -> Move:
        best_move = "S"
        best_distance = self._manhattan(start, goal)
        for move in MOVES:
            nxt = valid_next_pos(start, move, self.grid)
            if nxt == start:
                continue
            distance = self._manhattan(nxt, goal)
            if distance < best_distance:
                best_move = move
                best_distance = distance
        return best_move

    def _astar_next_move(self, start: Position, goal: Position) -> Move:
        key = (start, goal)
        if key in self._next_move_cache:
            return self._next_move_cache[key]
        if not is_valid_cell(start, self.grid) or not is_valid_cell(goal, self.grid):
            self._next_move_cache[key] = "S"
            return "S"

        heap: List[Tuple[int, int, Position]] = [(self._manhattan(start, goal), 0, start)]
        parent: Dict[Position, Tuple[Optional[Position], Move]] = {start: (None, "S")}
        best_cost: Dict[Position, int] = {start: 0}
        best_pos = start
        best_h = self._manhattan(start, goal)
        expansions = 0
        max_expansions = 2500

        while heap and expansions < max_expansions:
            _priority, cost, current = heapq.heappop(heap)
            if cost != best_cost.get(current):
                continue
            expansions += 1
            current_h = self._manhattan(current, goal)
            if current_h < best_h:
                best_h = current_h
                best_pos = current
            if current == goal:
                best_pos = current
                break

            for move, nxt in self._neighbors(current):
                new_cost = cost + 1
                if new_cost >= best_cost.get(nxt, INF):
                    continue
                best_cost[nxt] = new_cost
                parent[nxt] = (current, move)
                heapq.heappush(heap, (new_cost + self._manhattan(nxt, goal), new_cost, nxt))

        if best_pos == start:
            move = self._greedy_next_move(start, goal)
            self._next_move_cache[key] = move
            return move

        current = best_pos
        while True:
            previous, move = parent[current]
            if previous is None:
                self._next_move_cache[key] = "S"
                return "S"
            if previous == start:
                self._next_move_cache[key] = move
                return move
            current = previous

    def _bfs_distance(self, start: Position, goal: Position) -> int:
        if start == goal:
            return 0
        if self._large_map_mode:
            if not is_valid_cell(start, self.grid) or not is_valid_cell(goal, self.grid):
                return INF
            return self._manhattan(start, goal)
        key = (start, goal)
        if key in self._distance_cache:
            return self._distance_cache[key]
        if not is_valid_cell(start, self.grid) or not is_valid_cell(goal, self.grid):
            self._distance_cache[key] = INF
            return INF

        queue: deque[Tuple[Position, int]] = deque([(start, 0)])
        seen = {start}
        while queue:
            current, dist = queue.popleft()
            for _, nxt in self._neighbors(current):
                if nxt in seen:
                    continue
                if nxt == goal:
                    self._distance_cache[key] = dist + 1
                    return dist + 1
                seen.add(nxt)
                queue.append((nxt, dist + 1))
        self._distance_cache[key] = INF
        return INF

    def _move_between(self, start: Position, nxt: Position) -> Move:
        if start == nxt:
            return "S"
        for move in MOVES:
            if valid_next_pos(start, move, self.grid) == nxt:
                return move
        return "S"

    # ------------------------------------------------------------------
    # Task assignment
    # ------------------------------------------------------------------
    def _reward_proxy(self, order: Order) -> float:
        return {1: 1.0, 2: 2.0, 3: 3.0}[order.p] * r_base(order.w)

    def _delivery_score(self, shipper: Shipper, order: Order, t: int) -> float:
        distance = self._bfs_distance(shipper.position, (order.ex, order.ey))
        if distance >= INF:
            return -10**18
        eta = t + distance
        lateness = max(0, eta - order.et)
        slack = max(0, order.et - eta)
        urgency = 18.0 * order.p / max(1.0, slack + 1.0)
        return (
            7.0 * self._reward_proxy(order)
            + 25.0 * order.p
            + urgency
            - 2.0 * distance
            - 18.0 * lateness
            - 0.015 * slack
        )

    def _pickup_score(self, shipper: Shipper, order: Order, orders: Dict[int, Order], t: int) -> float:
        if not shipper.can_carry(order, orders):
            return -10**18
        pickup = (order.sx, order.sy)
        delivery = (order.ex, order.ey)
        d_pick = self._bfs_distance(shipper.position, pickup)
        d_deliver = self._bfs_distance(pickup, delivery)
        if d_pick >= INF or d_deliver >= INF:
            return -10**18

        eta = t + d_pick + d_deliver
        lateness = max(0, eta - order.et)
        slack = max(0, order.et - eta)

        # Detour penalty for currently carried packages
        detour_penalty = 0.0
        carried = [orders[oid] for oid in shipper.bag if oid in orders and not orders[oid].delivered]
        if carried:
            for co in carried:
                d_old = self._bfs_distance(shipper.position, (co.ex, co.ey))
                d_new = self._bfs_distance(pickup, (co.ex, co.ey))
                if d_old < INF and d_new < INF:
                    increase = d_pick + d_new - d_old
                    lateness_before = max(0, t + d_old - co.et)
                    lateness_after = max(0, t + d_old + increase - co.et)
                    detour_penalty += 20.0 * (lateness_after - lateness_before) * co.p

        hotspot_bias = 4.0 if order.p == 3 else 0.0
        return (
            7.5 * self._reward_proxy(order)
            + 30.0 * order.p
            + 8.0 * order.p / max(1.0, slack + 1.0)
            + hotspot_bias
            - 1.4 * d_pick
            - 0.55 * d_deliver
            - 16.0 * lateness
            - 0.01 * slack
            - detour_penalty
        )

    def _combined_delivery_score(self, shipper: Shipper, target_orders: List[Order], t: int) -> float:
        if not target_orders:
            return -10**18
        dest = (target_orders[0].ex, target_orders[0].ey)
        distance = self._bfs_distance(shipper.position, dest)
        if distance >= INF:
            return -10**18
        
        score = -2.0 * distance
        for order in target_orders:
            eta = t + distance
            lateness = max(0, eta - order.et)
            slack = max(0, order.et - eta)
            urgency = 18.0 * order.p / max(1.0, slack + 1.0)
            score += (
                7.0 * self._reward_proxy(order)
                + 25.0 * order.p
                + urgency
                - 18.0 * lateness
                - 0.015 * slack
            )
        return score

    def _assign_targets(self, obs: dict) -> Dict[int, Tuple[Position, int]]:
        t = int(obs["t"])
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)

        candidate_pairs = []
        for shipper in shippers:
            # Deliveries: group by destination
            carried_by_dest = {}
            for oid in shipper.bag:
                order = orders.get(oid)
                if order is not None and not order.delivered:
                    dest = (order.ex, order.ey)
                    carried_by_dest.setdefault(dest, []).append(order)
            
            for dest, target_orders in carried_by_dest.items():
                score = self._combined_delivery_score(shipper, target_orders, t)
                candidate_pairs.append((score, shipper.id, ('deliver', target_orders[0].id, dest)))
            
            # Pickups
            if len(shipper.bag) < shipper.K_max:
                w_carried = sum(orders[oid].w for oid in shipper.bag if oid in orders)
                for order in orders.values():
                    if order.picked or order.delivered:
                        continue
                    if w_carried + order.w <= shipper.W_max:
                        score = self._pickup_score(shipper, order, orders, t)
                        candidate_pairs.append((score, shipper.id, ('pickup', order.id, (order.sx, order.sy))))

        # Sort by score descending
        candidate_pairs.sort(key=lambda x: (x[0], -x[1]), reverse=True)

        targets: Dict[int, Tuple[Position, int]] = {}
        assigned_shippers = set()
        assigned_pickups = set()
        self._shipper_target_order = {}

        for score, sid, target_info in candidate_pairs:
            if sid in assigned_shippers:
                continue
            op_type, oid, goal = target_info
            if op_type == 'pickup':
                if oid in assigned_pickups:
                    continue
                assigned_pickups.add(oid)
                targets[sid] = (goal, 1)
                assigned_shippers.add(sid)
                self._shipper_target_order[sid] = oid
            elif op_type == 'deliver':
                targets[sid] = (goal, 2)
                assigned_shippers.add(sid)
                self._shipper_target_order[sid] = oid

        for shipper in shippers:
            if shipper.id not in targets:
                targets[shipper.id] = (shipper.position, 0)
                self._shipper_target_order[shipper.id] = -1

        return targets

    # ------------------------------------------------------------------
    # Bounded CBS
    # ------------------------------------------------------------------
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
    ) -> Path:
        if start == goal:
            return [start]
        if self._large_map_mode:
            path = [start]
            current = start
            for tau in range(1, horizon + 1):
                astar_move = self._astar_next_move(current, goal)
                astar_next = valid_next_pos(current, astar_move, self.grid)
                if not self._violates_constraints(sid, current, astar_next, tau, vertex_constraints, edge_constraints):
                    best_next = astar_next
                    best_dist = self._manhattan(astar_next, goal)
                else:
                    best_next = current
                    best_dist = self._manhattan(current, goal)
                for move in WAIT_MOVES:
                    nxt = valid_next_pos(current, move, self.grid)
                    if self._violates_constraints(sid, current, nxt, tau, vertex_constraints, edge_constraints):
                        continue
                    dist = self._manhattan(nxt, goal)
                    if dist < best_dist or (best_next == current and dist == best_dist):
                        best_next = nxt
                        best_dist = dist
                path.append(best_next)
                current = best_next
                if current == goal:
                    break
            return path

        queue: deque[Tuple[Position, int]] = deque([(start, 0)])
        parent: Dict[Tuple[Position, int], Tuple[Optional[Tuple[Position, int]], Position]] = {
            (start, 0): (None, start)
        }
        best_state = (start, 0)
        best_dist = self._bfs_distance(start, goal)

        while queue:
            current, tau = queue.popleft()
            current_dist = self._bfs_distance(current, goal)
            if current_dist < best_dist:
                best_dist = current_dist
                best_state = (current, tau)

            if current == goal:
                best_state = (current, tau)
                break
            if tau >= horizon:
                continue

            for move in WAIT_MOVES:
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

    def _path_at(self, path: Path, tau: int) -> Position:
        if not path:
            raise ValueError("empty path")
        return path[tau] if tau < len(path) else path[-1]

    def _find_conflict(self, paths: Dict[int, Path], horizon: int) -> Optional[Tuple[str, int, int, Position, Position, int]]:
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

    def _cbs_paths(self, obs: dict, targets: Dict[int, Tuple[Position, int]]) -> Dict[int, Path]:
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)
        shipper_by_id = {s.id: s for s in shippers}
        horizon = min(18, max(6, 2 * int(obs.get("N", 10))))
        vertex_constraints: set[Tuple[int, Position, int]] = set()
        edge_constraints: set[Tuple[int, Position, Position, int]] = set()

        paths: Dict[int, Path] = {}
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

        max_repairs = 12 if self._large_map_mode else 80
        for _ in range(max_repairs):
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
            before = obs
            obs, _, done, _ = self.env.step(actions)
            self._trace_step(before, actions, obs)
            if done:
                break

        return self.env.result(
            self.method_name,
            elapsed_sec=time.time() - start_time,
        )
