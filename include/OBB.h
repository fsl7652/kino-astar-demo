#pragma once
#define _USE_MATH_DEFINES
#include <array>
#define SDL_MAIN_HANDLED
#define SDL_DISABLE_IMPLICIT_LINKING
#include <SDL3/SDL.h>
#include <limits>

struct CollisionResult {
    bool collided;
    SDL_FPoint normal;
    float depth;
};


class OBB
{
public:
    SDL_FPoint center;   
    float half_w;         
    float half_h;        
    float angle;

    OBB(float x=0,float y=0,float w=1,float h=1,float angle=0)
        : center{ x, y }, half_w(w/2), half_h(h/2), angle(angle) {}

    std::array<SDL_FPoint,4> getCorners() const;

    CollisionResult collision(const OBB& obj) const;

    CollisionResult expandedCollision(const OBB& obj, float margin = 0.2f) const;

private:
    SDL_FPoint normalFromEdge(const SDL_FPoint& p1, const SDL_FPoint& p2) const;
    void projectNormal(const std::array<SDL_FPoint, 4>& pts,const SDL_FPoint& normal,float& min, float& max) const;
};
