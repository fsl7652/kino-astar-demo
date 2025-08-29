#include "wall.h"

void Wall::render(SDL_Renderer* renderer, float scale, SDL_Color colour) const {
    auto pts = box.getCorners();
    SDL_FPoint loop[5];
    for (int i = 0; i < 4; i++) {
        loop[i].x = pts[i].x * scale;
        loop[i].y = pts[i].y * scale;
    }
    loop[4] = loop[0];
    SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);
    SDL_RenderLines(renderer, loop, 5);
}
