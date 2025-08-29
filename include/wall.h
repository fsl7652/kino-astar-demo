#pragma once
#include <array>
#include "OBB.h"

/// @brief Wall structure containing an OBB and rendering function
/// @param box The oriented bounding box representing the wall
struct Wall {
    OBB box;

    Wall(float x, float y, float w, float h, float angle = 0.0f)
        : box(x, y, w, h, angle) {}

    void render(SDL_Renderer* renderer, float scale = 100.0f, SDL_Color colour = { 255, 0, 0, 255 }) const;
}; 

