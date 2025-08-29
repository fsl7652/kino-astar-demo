#pragma once
#define _USE_MATH_DEFINES
#include <array>
#define SDL_MAIN_HANDLED
#define SDL_DISABLE_IMPLICIT_LINKING
#include <SDL3/SDL.h>
#include <limits>
#include <cmath>
#include <cfloat>


/// @brief Result of a collision check between two OBBs
/// @param collided Whether a collision occurred
/// @param normal The collision normal vector
/// @param depth The penetration depth of the collision
struct CollisionResult {
    bool collided;
    SDL_FPoint normal;
    float depth;
};

/// @brief Oriented Bounding Box (OBB) for 2D collision detection
/// @param center Center position of the OBB
/// @param half_w Half the width of the OBB
/// @param half_h Half the height of the OBB
/// @param angle Rotation angle of the OBB in radians
/// @note Uses Separating Axis Theorem (SAT) for collision detection
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
