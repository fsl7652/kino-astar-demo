#include "goal.h"

//renders the goal box as a green rectangle
void Goal::render(SDL_Renderer* renderer, float scale) const {
    auto pts = box.getCorners();
    SDL_FPoint loop[5];
    for (int i = 0; i < 4; i++) {
        loop[i].x = pts[i].x * scale;
        loop[i].y = pts[i].y * scale;
    }
    loop[4] = loop[0];
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderLines(renderer, loop, 5);
}