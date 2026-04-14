#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <map>
#include <set>

struct CellPos {
  int row;
  int col;
  CellPos(int r = 0, int c = 0) : row(r), col(c) {}

    bool operator<(const CellPos& other) const {
    if (row != other.row) return row < other.row;
    return col < other.col;
  }
};



class Mosiac {
public:


    Mosiac();

    // Get target cell positions for a given color index
    std::set<CellPos> getTargetCells(int colorIndex); 

    // Add a cell position to a color index
    void addCellToColor(int colorIndex, const CellPos& cellPos);

    // Remove a cell position from a color index
    void removeCellFromColor(int colorIndex, const CellPos& cellPos);

    // Accept placement of a tile with color index at cell position, return true if it matches target
    bool placeTile(int colorIndex, const CellPos& cellPos);


    // Debug function to print the mosaic mapping
    void debug() const {
        Serial.println("Mosaic Mapping:");
        for (const auto& pair : mosiac) {
            int colorIndex = pair.first;
            const std::set<CellPos>& positions = pair.second;
            Serial.print("Color Index ");
            Serial.print(colorIndex);
            Serial.print(": ");
            for (const CellPos& pos : positions) {
                Serial.print("(");
                Serial.print(pos.row);
                Serial.print(", ");
                Serial.print(pos.col);
                Serial.print(") ");
            }
            Serial.println();
        }
    }
private: 
    // Init function

    
    // Color ID mapping to cell positions
    std::map<int, std::set<CellPos>> mosiac;


};
