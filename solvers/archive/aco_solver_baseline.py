from __future__ import annotations

import random
import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple

from env import DeliveryEnv, Order, Shipper, is_valid_cell, r_base, valid_next_pos
from solvers.solver import Solver


Move = str
Position = Tuple[int, int]
Action = Tuple[Move, object]

INF = 10**9
MOVES: Tuple[Move, ...] = ("U", "D", "L", "R")


class ACOSolver(Solver):
    """
    Rolling-horizon Ant Colony Optimization baseline.

    The solver samples several pickup assignments for idle shippers from
    pheromone-weighted probabilities, evaluates them with a reward/deadline
    proxy, updates pheromone, and executes only the first move.
    """

    method_name = "ACO"

    def __init__(self, env: DeliveryEnv):
        super().__init__(env)
        self._distance_cache: Dict[Tuple[Position, Position], int] = {}
        self._next_move_cache: Dict[Tuple[Position, Position], Move] = {}
        self._pheromone: Dict[Tuple[int, int], float] = {}
        self._rng = random.Random(20240527)

    # ------------------------------------------------------------------
    # BFS helpers
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

    def _heuristic(self, shipper: Shipper, order: Order, t: int) -> float:
        d1 = self._distance(shipper.position, (order.sx, order.sy))
        d2 = self._distance((order.sx, order.sy), (order.ex, order.ey))
        if d1 >= INF or d2 >= INF:
            return 0.0
        eta = t + d1 + d2
        lateness = max(0, eta - order.et)
        slack = max(0, order.et - eta)
        return max(
            0.001,
            (self._reward_proxy(order) * (1.0 + 0.30 * order.p) + 0.04 * slack)
            / (1.0 + d1 + 0.35 * d2 + 3.0 * lateness),
        )

    def _assignment_score(self, plan: Dict[int, int], shippers: Dict[int, Shipper], orders: Dict[int, Order], t: int) -> float:
        score = 0.0
        for sid, oid in plan.items():
            shipper = shippers[sid]
            order = orders[oid]
            d1 = self._distance(shipper.position, (order.sx, order.sy))
            d2 = self._distance((order.sx, order.sy), (order.ex, order.ey))
            eta = t + d1 + d2
            lateness = max(0, eta - order.et)
            score += 6.0 * self._reward_proxy(order) + 12.0 * order.p - d1 - 0.4 * d2 - 6.0 * lateness
        return score

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
        return max(
            carried,
            key=lambda o: (
                self._reward_proxy(o) - 2.5 * max(0, self._distance(shipper.position, (o.ex, o.ey)) - max(0, o.et)),
                -self._distance(shipper.position, (o.ex, o.ey)),
                -o.et,
                o.p,
            ),
        )

    def _delivery_action(self, shipper: Shipper, order: Order) -> Action:
        goal = (order.ex, order.ey)
        move, nxt = self._move_towards(shipper, goal)
        return (move, 2) if nxt == goal else (move, 0)

    def _pickup_action(self, shipper: Shipper, order: Order) -> Action:
        goal = (order.sx, order.sy)
        move, nxt = self._move_towards(shipper, goal)
        return (move, 1) if nxt == goal else (move, 0)

    # ------------------------------------------------------------------
    # ACO assignment
    # ------------------------------------------------------------------
    def _candidate_orders(self, shippers: List[Shipper], orders: Dict[int, Order], t: int) -> List[Order]:
        candidates: List[Order] = []
        for order in orders.values():
            if order.picked or order.delivered:
                continue
            if not any(s.can_carry(order, orders) and self._heuristic(s, order, t) > 0 for s in shippers):
                continue
            candidates.append(order)

        candidates.sort(
            key=lambda o: (
                -o.p,
                o.et,
                -self._reward_proxy(o),
                o.id,
            )
        )
        return candidates[: min(28, len(candidates))]

    def _weighted_choice(self, weighted_items: List[Tuple[Order, float]]) -> Optional[Order]:
        total = sum(max(0.0, w) for _, w in weighted_items)
        if total <= 0:
            return None
        pick = self._rng.random() * total
        acc = 0.0
        for order, weight in weighted_items:
            acc += max(0.0, weight)
            if acc >= pick:
                return order
        return weighted_items[-1][0]

    def _construct_ant_plan(
        self,
        shippers: List[Shipper],
        candidates: List[Order],
        orders: Dict[int, Order],
        t: int,
    ) -> Dict[int, int]:
        plan: Dict[int, int] = {}
        used: set[int] = set()

        for shipper in shippers:
            choices: List[Tuple[Order, float]] = []
            for order in candidates:
                if order.id in used or not shipper.can_carry(order, orders):
                    continue
                heuristic = self._heuristic(shipper, order, t)
                if heuristic <= 0:
                    continue
                tau = self._pheromone.get((shipper.id, order.id), 1.0)
                weight = (tau ** 1.1) * (heuristic ** 2.2)
                choices.append((order, weight))

            selected = self._weighted_choice(choices)
            if selected is not None:
                plan[shipper.id] = selected.id
                used.add(selected.id)

        return plan

    def _aco_assign(self, shippers: List[Shipper], orders: Dict[int, Order], t: int) -> Dict[int, int]:
        candidates = self._candidate_orders(shippers, orders, t)
        if not candidates or not shippers:
            return {}

        shipper_by_id = {s.id: s for s in shippers}
        iterations = 8
        ants = max(10, 3 * len(shippers))
        best_plan: Dict[int, int] = {}
        best_score = -10**18

        for _ in range(iterations):
            iteration_best: Dict[int, int] = {}
            iteration_score = -10**18

            for _ant in range(ants):
                plan = self._construct_ant_plan(shippers, candidates, orders, t)
                score = self._assignment_score(plan, shipper_by_id, orders, t)
                if score > iteration_score:
                    iteration_score = score
                    iteration_best = plan
                if score > best_score:
                    best_score = score
                    best_plan = plan

            for key in list(self._pheromone.keys()):
                self._pheromone[key] *= 0.82
                if self._pheromone[key] < 0.05:
                    del self._pheromone[key]

            deposit = max(0.1, iteration_score / 100.0)
            for sid, oid in iteration_best.items():
                self._pheromone[(sid, oid)] = min(
                    20.0,
                    self._pheromone.get((sid, oid), 1.0) + deposit,
                )

        return best_plan

    def _fallback_pickup(
        self,
        shipper: Shipper,
        orders: Dict[int, Order],
        reserved: set[int],
        t: int,
    ) -> Optional[Order]:
        best: Optional[Order] = None
        best_score = -1.0
        for order in orders.values():
            if order.id in reserved or not shipper.can_carry(order, orders):
                continue
            score = self._heuristic(shipper, order, t)
            if score > best_score or (score == best_score and order.id < (best.id if best else INF)):
                best_score = score
                best = order
        return best

    def _decide_actions(self, obs: dict) -> Dict[int, Action]:
        t = int(obs["t"])
        orders: Dict[int, Order] = obs["orders"]
        shippers: List[Shipper] = sorted(obs["shippers"], key=lambda s: s.id)

        actions: Dict[int, Action] = {}
        idle: List[Shipper] = []

        for shipper in shippers:
            delivery = self._delivery_order(shipper, orders)
            if delivery is None:
                idle.append(shipper)
            else:
                actions[shipper.id] = self._delivery_action(shipper, delivery)

        assigned = self._aco_assign(idle, orders, t)
        reserved: set[int] = set()
        for shipper in idle:
            order = orders.get(assigned.get(shipper.id, -1))
            if order is None or order.id in reserved or not shipper.can_carry(order, orders):
                order = self._fallback_pickup(shipper, orders, reserved, t)

            if order is None:
                actions[shipper.id] = ("S", 0)
                continue

            reserved.add(order.id)
            actions[shipper.id] = self._pickup_action(shipper, order)

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
