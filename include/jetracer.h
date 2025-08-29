#pragma once
#define _USE_MATH_DEFINES
#include "OBB.h"
#include <vector>
#include "kino_search.h"

class JetRacer {
public:
    float x, y;   //position
    float yaw;    //heading
    SDL_FPoint v;      //velocity
    SDL_FPoint look; //lookahead
    bool pursuit;

    float wheelbase;   //distance between front and rear axle
    float length;      //length
    float width;       //width
    SDL_Color colour;   //colour for rendering

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
