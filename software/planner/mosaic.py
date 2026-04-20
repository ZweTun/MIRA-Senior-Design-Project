from dataclasses import dataclass, field
from typing import Dict, Set


from planner.models import CellPos
from planner.models import FeedbackPacket
from planner.models import GuidanceState

COLOR_TO_ID = {
    "red": 7,
    "green": 8,
    "blue": 9,
    "magenta": 10,
    "cyan": 11,
    "yellow": 12,
    "brown": 13,
    "black": 14,
    "orange": 15,
    "empty": 255


}

TAG_TO_COLOR_ID = {
    7: COLOR_TO_ID["red"],
    8: COLOR_TO_ID["green"],
    9: COLOR_TO_ID["blue"],

    10: COLOR_TO_ID["magenta"],
    11: COLOR_TO_ID["cyan"],
    12: COLOR_TO_ID["yellow"],
    13: COLOR_TO_ID["brown"],
    14: COLOR_TO_ID["black"],
    15: COLOR_TO_ID["orange"],
    255: COLOR_TO_ID["empty"]
}

ID_TO_COLOR = {v: k for k, v in COLOR_TO_ID.items()}

@dataclass
class Mosaic:


    # color_index -> set of target cell positions
    targets_by_color: Dict[int, Set[CellPos]] = field(default_factory=dict)

    def get_target_cells(self, color) -> Set[CellPos]:
        if isinstance(color, str):
            color_index = COLOR_TO_ID.get(color)
        else:
            color_index = color

        if color_index is None:
            return set()

        # print(f"Getting target cells for color index {color_index}")
        return self.targets_by_color.get(color_index, set())

    def add_cell_to_color(self, color_index: int, cell_pos: CellPos) -> None:
        if color_index not in self.targets_by_color:
            self.targets_by_color[color_index] = set()
        self.targets_by_color[color_index].add(cell_pos)

    def remove_cell_from_color(self, color_index: int, cell_pos: CellPos) -> None:
        if color_index in self.targets_by_color:
            self.targets_by_color[color_index].discard(cell_pos)

    def place_tile(self, color_index: int, cell_pos: CellPos) -> bool:
        if color_index in self.targets_by_color and cell_pos in self.targets_by_color[color_index]:
            self.targets_by_color[color_index].remove(cell_pos)
            return True
        return False
  

    def get_all_targets(self) -> Set[CellPos]:
        all_targets = set()
        for targets in self.targets_by_color.values():
            all_targets.update(targets)
        return all_targets