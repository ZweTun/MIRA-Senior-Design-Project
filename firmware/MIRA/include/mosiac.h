#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <map>
#include <set>

struct CellPos {
  int posX;
  int posY;
  CellPos(int x = 0, int y = 0) : posX(x), posY(y) {}

    bool operator<(const CellPos& other) const {
    if (posX != other.posX) return posX < other.posX;
    return posY < other.posY;
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
                Serial.print(pos.posX);
                Serial.print(", ");
                Serial.print(pos.posY);
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
