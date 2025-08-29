#pragma once
#define _USE_MATH_DEFINES
#include "OBB.h"
#include <vector>
#include <cmath>
#include "kino_search.h"

/// @brief Simple car model with position, velocity, and rendering
/// @param x X position
/// @param y Y position
/// @param yaw Heading angle in radians
/// @param v Velocity vector 
/// @param look Lookahead point for path following
/// @param pursuit Whether the car is currently following a path
/// @param wheelbase Distance between front and rear axles
/// @param length Car length for OBB
/// @param width Car width for OBB
/// @param colour Car colour for rendering
/// @param max_speed Maximum forward speed
/// @param max_acceleration Maximum acceleration
/// @param max_deceleration Maximum deceleration (braking)
/// @param max_reverse_speed Maximum reverse speed
/// @param friction Friction coefficient for slowing down when not accelerating
/// @param current_path Current path being followed
/// @param current_waypoint Index of the next waypoint in the path
/// @param lookahead_distance Distance ahead of the car to look for path following
/// @note Uses a simple bicycle model for motion and pure pursuit for path following


class JetRacer {
public:
    float x, y;   
    float yaw;    
    SDL_FPoint v;     
    SDL_FPoint look; 
    bool pursuit;

    float wheelbase;   
    float length;      
    float width;       
    SDL_Color colour;   

    float max_speed;        
    float max_acceleration;   
    float max_deceleration;   
    float max_reverse_speed;
    float friction;

    JetRacer(float x0 = 0, float y0 = 0, float yaw0 = 0);

    void update(float dt, float throttle, float steering);

    void render(SDL_Renderer* renderer, float scale = 100.0f) const;

    void reset(float x0, float y0, float yaw0, SDL_FPoint v0);

    void purePursuitPath(const std::vector<State>& path, const OBB &goal, float dt);
    void setPath(const std::vector<State>& path) { current_path = path; current_waypoint = 0; }
    std::vector<State> getPath() { return current_path; };
    void clearPath() { current_path.clear(); current_waypoint = 0; }
    bool getPursuit() { return pursuit; }

    bool pathIntersectsObstacles(const std::vector<OBB>& obstacles);

    SDL_FRect getRect();
    void stop();
    void collide();

    OBB getOBB();
private:
    std::vector<State> current_path;
    int current_waypoint = 1;
    float lookahead_distance = 1.0f;
};
