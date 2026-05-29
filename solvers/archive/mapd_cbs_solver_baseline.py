from __future__ import annotations

import time
from collections import deque
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

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------
    def _neighbors(self, pos: Position) -> Iterable[Tuple[Move, Position]]:
        for move in MOVES:
            nxt = valid_next_pos(pos, move, self.grid)
            if nxt != pos:
                yield move, nxt

    def _bfs_distance(self, start: Position, goal: Position) -> int:
        if start == goal:
            return 0
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

    def _delivery_target(self, shipper: Shipper, orders: Dict[int, Order], t: int) -> Optional[Order]:
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
                max(0, o.et - t),
                self._bfs_distance(shipper.position, (o.ex, o.ey)),
                -o.p,
                o.id,
            ),
        )

    def _pickup_score(self, shipper: Shipper, order: Order, t: int) -> float:
        d1 = self._bfs_distance(shipper.position, (order.sx, order.sy))
        d2 = self._bfs_distance((order.sx, order.sy), (order.ex, order.ey))
        if d1 >= INF or d2 >= INF:
            return -10**18
        eta = t + d1 + d2
        lateness = max(0, eta - order.et)
        return (
            5.0 * self._reward_proxy(order)
            + 10.0 * order.p
            - 1.1 * d1
            - 0.35 * d2
            - 5.0 * lateness
        )

    def _pickup_target(
        self,
        shipper: Shipper,
        orders: Dict[int, Order],
        reserved_orders: set[int],
        t: int,
    ) -> Optional[Order]:
        best: Optional[Order] = None
        best_score = -10**18
        for order in orders.values():
            if order.id in reserved_orders or not shipper.can_carry(order, orders):
                continue
            score = self._pickup_score(shipper, order, t)
            if score > best_score or (score == best_score and order.id < (best.id if best else INF)):
                best = order
                best_score = score
        return best

    def _assign_targets(self, obs: dict) -> Dict[int, Tuple[Position, int]]:
        t = int(obs["t"])
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)

        targets: Dict[int, Tuple[Position, int]] = {}
        reserved_orders: set[int] = set()

        for shipper in shippers:
            delivery = self._delivery_target(shipper, orders, t)
            if delivery is not None:
                targets[shipper.id] = ((delivery.ex, delivery.ey), 2)
                continue

            pickup = self._pickup_target(shipper, orders, reserved_orders, t)
            if pickup is not None:
                reserved_orders.add(pickup.id)
                targets[shipper.id] = ((pickup.sx, pickup.sy), 1)
            else:
                targets[shipper.id] = (shipper.position, 0)

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

        for _ in range(80):
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

        actions: Dict[int, Action] = {}
        for shipper in shippers:
            path = paths.get(shipper.id, [shipper.position])
            next_pos = path[1] if len(path) > 1 else path[0]
            move = self._move_between(shipper.position, next_pos)
            goal, op = targets[shipper.id]
            actions[shipper.id] = (move, op) if next_pos == goal else (move, 0)
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
