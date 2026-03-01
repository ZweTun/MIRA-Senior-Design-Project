#include "mosiac.h"

Mosiac::Mosiac() {
 
}

std::set<CellPos> Mosiac::getTargetCells(int colorIndex) {
    if (mosiac.find(colorIndex) != mosiac.end()) {
        return mosiac[colorIndex];
    }
    return std::set<CellPos>(); // empty set if color index not found
}

void Mosiac::addCellToColor(int colorIndex, const CellPos& cellPos) {
    mosiac[colorIndex].insert(cellPos);
}

void Mosiac::removeCellFromColor(int colorIndex, const CellPos& cellPos) {
    if (mosiac.find(colorIndex) != mosiac.end()) {
        mosiac[colorIndex].erase(cellPos);
    }
}

bool Mosiac::placeTile(int colorIndex, const CellPos& cellPos) {
    if (mosiac.find(colorIndex) != mosiac.end()) {
        return mosiac[colorIndex].count(cellPos) > 0; // true if cellPos is a target for this color
    }
    return false; // color index not found
}

