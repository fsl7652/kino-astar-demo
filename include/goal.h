#include <array>
#include "OBB.h"

/// @brief Goal structure containing an OBB and rendering function
/// @param box The oriented bounding box representing the goal area
/// @param display Whether to render the goal box

struct Goal {
    OBB box;
    bool display;

    Goal() : box(0.0f, 0.0f, 0.0f, 0.0f, 0.0f) { }
    Goal(float x, float y, float w, float h, float angle = 0.0f, bool display = false)
        : box(x, y, w, h, angle) {}

    void render(SDL_Renderer* renderer, float scale = 100.0f) const;
};
