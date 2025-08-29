#include "jetracer.h"
#include <cmath>

JetRacer::JetRacer(float x0, float y0, float yaw0)
    : x(x0), y(y0), yaw(yaw0), v({0.0,0.0}),
    wheelbase(0.15f), length(0.25f), width(0.19f), 
    max_speed(2.0f), max_acceleration(0.75f), 
    max_deceleration(1.0f), max_reverse_speed(0.5f), friction(0.25f), 
    pursuit(false), look({0.0f, 0.0f})
{
    colour = { 255, 0, 0, 255 };
}

void JetRacer::update(float dt, float throttle, float steering) {

    static float cur_steer = 0.0f; 
    float steer_rate = 2.0f; 
    cur_steer += (steering - cur_steer) * steer_rate * dt;
    float accel = throttle * max_acceleration;
    accel = std::min(max_acceleration, std::max(-max_deceleration, accel));

    float current_speed = std::sqrt(v.x * v.x + v.y * v.y);
    float speed_change = accel * dt;

    if (current_speed + speed_change > max_speed) {
        speed_change = max_speed - current_speed;
    }
    else if (current_speed + speed_change < -max_reverse_speed) {
        speed_change = -max_reverse_speed - current_speed;
    }

    if (std::abs(cur_steer) < 0.001f) {
        v.x += speed_change * std::cos(yaw);
        v.y += speed_change * std::sin(yaw);
    }
    else {
        float turning_radius = wheelbase / std::tan(cur_steer);
        float angular_velocity = current_speed / turning_radius;

        yaw += angular_velocity * dt;

        float new_speed = current_speed + speed_change;
        v.x = new_speed * std::cos(yaw);
        v.y = new_speed * std::sin(yaw);
    }

    v.x /= 2.0f;
    v.y /= 2.0f;

    x += v.x * dt;
    y += v.y * dt;

    v.x *= (1.0f - friction * dt );
    v.y *= (1.0f - friction * dt );
}

void JetRacer::render(SDL_Renderer* renderer, float scale) const {
    SDL_FPoint corners[5];
    float cos_y = std::cos(yaw);
    float sin_y = std::sin(yaw);

    float hl = length / 2.0f;
    float hw = width / 2.0f;

    float cx[4] = { hl,  hl, -hl, -hl };
    float cy[4] = { hw, -hw, -hw,  hw };

    for (int i = 0; i < 4; i++) {
        float wx = x + (cx[i] * cos_y - cy[i] * sin_y);
        float wy = y + (cx[i] * sin_y + cy[i] * cos_y);
        corners[i].x = wx * scale;
        corners[i].y = wy * scale;
    }
    corners[4] = corners[0];

    SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);
    SDL_RenderLines(renderer, corners, 5);

    if (look.x != 0.0f && look.y != 0.0f) {
        RenderCircle::drawCircle(renderer, {look.x * scale, look.y * scale}, 5, { 255, 0, 255, 255 });
    }

}

void JetRacer::reset(float x0, float y0, float yaw0, SDL_FPoint v0) {
    x = x0;
    y = y0;
    yaw = yaw0;
    v = v0;
}

void JetRacer::stop() {
    v = { 0.0f ,0.0f};
    clearPath();
}

OBB JetRacer::getOBB() {
    return OBB(x, y, length, width, yaw);
}

void JetRacer::purePursuitPath(const std::vector<State>& path, const OBB& goal_box, float dt) {
    if (path.empty() || current_waypoint >= path.size())
        return;

    if (getOBB().collision(goal_box).collided) {
        reset(x, y, yaw, {0.0f,0.0f});
        SDL_Log("Reached goal, stopping pursuit.");
        look = { 0.0f, 0.0f };
        pursuit = false;
        return;
    }
    pursuit = true;
    double dx_goal = path.back().x - x;
    double dy_goal = path.back().y - y;
    double goal_dist = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
    float effective_look = std::min(lookahead_distance, (float)goal_dist);

    while (current_waypoint < path.size() - 1) {
        double dx = path[current_waypoint].x - x;
        double dy = path[current_waypoint].y - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > effective_look)
            break;
        current_waypoint++;
    }

    State look_pt = path.back();
    float total_dist = 0.0f;
    int look_idx = current_waypoint;

    if (goal_dist <= effective_look) {
        look_pt = path.back(); 
    }
    else {
        for (int i = current_waypoint; i < path.size() - 1; i++) {
            float seg_length = std::sqrt(
                std::pow(path[i + 1].x - path[i].x, 2) +
                std::pow(path[i + 1].y - path[i].y, 2)
            );

            if (total_dist + seg_length >= effective_look) {
                float rem = effective_look - total_dist;
                float ratio = rem / seg_length;

                look_pt.x = path[i].x + ratio * (path[i + 1].x - path[i].x);
                look_pt.y = path[i].y + ratio * (path[i + 1].y - path[i].y);
                look_idx = i;
                break;
            }

            total_dist += seg_length;
            look_pt = path[i + 1];
        }
    }
    look = { float(look_pt.x), float(look_pt.y) };
    SDL_FPoint rel_pos = { look.x - x, look.y - y };
    double alpha = std::atan2(rel_pos.y, rel_pos.x) - yaw;
    double steer = std::atan2(2.0 * wheelbase * std::sin(alpha), effective_look);
    steer = std::min(0.5, std::max(steer, -0.5));

    double curv = std::abs(steer) / wheelbase;
    double target_vel = max_speed / (3.0 * curv + 1e-5); 
    double vel_err = target_vel - std::sqrt(v.x * v.x + v.y * v.y);
    double throttle = 0.5 + 0.3 * vel_err;

    /*SDL_Log("Car pos: (%.2f, %.2f), yaw: %.2f | Lookahead: (%.2f, %.2f), waypoint %d/%d",
        x, y, yaw,
        look_pt.x, look_pt.y,
        current_waypoint, (int)path.size());*/

    update(dt, throttle, steer);
}

bool JetRacer::pathIntersectsObstacles(const std::vector<OBB>& obstacles) {
    for (const auto& state : getPath()) {
        for (const auto& obs : obstacles) {
            if (getOBB().collision(obs).collided) {
                return true;
            }
        }
    }
    return false;
}

