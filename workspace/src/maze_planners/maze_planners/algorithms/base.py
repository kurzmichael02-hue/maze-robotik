from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class PlanResult:
    path: List[Tuple[int, int]] = field(default_factory=list)
    expanded: int = 0           # wie viele zellen besucht/expandiert wurden
    runtime_ms: float = 0.0
    success: bool = False

    @property
    def length(self):
        return max(0, len(self.path) - 1)


class Planner:
    name = 'base'

    def plan(self, grid, start=None, goal=None) -> PlanResult:
        raise NotImplementedError
