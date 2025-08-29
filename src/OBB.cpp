#include "OBB.h"

//calculates the four corners of the OBB based on center, half-width, half-height and rotation angle
std::array<SDL_FPoint, 4> OBB::getCorners() const {
    std::array<SDL_FPoint, 4> pts{};
    float cos_y = std::cos(angle);
    float sin_y = std::sin(angle);
    float local_x[4] = { half_w,  half_w, -half_w, -half_w };
    float local_y[4] = { half_h,  -half_h, -half_h, half_h };

    for (int i = 0; i < 4; i++) {
        float world_x = center.x + (local_x[i] * cos_y - local_y[i] * sin_y);
        float world_y = center.y + (local_x[i] * sin_y + local_y[i] * cos_y);
        pts[i] = { world_x, world_y };
    }
    return pts;
}
//OBB collision detection using Separating Axis Theorem (SAT)
CollisionResult OBB::collision(const OBB& obj) const {
    auto obj1corners = getCorners();
    auto obj2corners = obj.getCorners();

    CollisionResult result;
    result.collided = true;   
    result.depth = FLT_MAX;
    result.normal = { 0,0 };

    for (int obj_count = 0; obj_count < 2; obj_count++)
    {
        const auto& pts = (obj_count == 0 ? obj1corners : obj2corners);
        for (int i = 0; i < 4; i++)
        {
            SDL_FPoint normal = normalFromEdge(pts[i], pts[(i+1) % 4]);
            //project both OBBs onto the normal and check for overlap
            float min1, max1, min2, max2;
            projectNormal(obj1corners,normal, min1, max1);
            projectNormal(obj2corners, normal, min2, max2);
            if (max1 < min2 || max2 < min1) {
                result.collided = false;
                return result; 
            }
            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = normal;

                SDL_FPoint d{ obj.center.x - center.x, obj.center.y - center.y };
                float dot = d.x * normal.x + d.y * normal.y;
                if (dot > 0) {
                    result.normal.x = -normal.x;
                    result.normal.y = -normal.y;
                }
            }
        }
    }
    return result;
}
//expanded collision check with margin for pathfinding clearance
CollisionResult OBB::expandedCollision(const OBB& obj, float margin) const {
    auto obj1corners = getCorners();
    auto conf_obj = OBB(obj.center.x, obj.center.y, (obj.half_w * 2) + margin, (obj.half_h * 2) + margin, obj.angle);
    auto obj2corners = conf_obj.getCorners();
    CollisionResult result;
    result.collided = true;
    result.depth = FLT_MAX;
    result.normal = { 0,0 };

    for (int obj_count = 0; obj_count < 2; obj_count++)
    {
        const auto& pts = (obj_count == 0 ? obj1corners : obj2corners);
        for (int i = 0; i < 4; i++)
        {
            SDL_FPoint normal = normalFromEdge(pts[i], pts[(i + 1) % 4]);

            float min1, max1, min2, max2;
            projectNormal(obj1corners, normal, min1, max1);
            projectNormal(obj2corners, normal, min2, max2);
            if (max1 < min2 || max2 < min1) {
                result.collided = false;
                return result;
            }
            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = normal;

                SDL_FPoint d{ obj.center.x - center.x, obj.center.y - center.y };
                float dot = d.x * normal.x + d.y * normal.y;
                if (dot > 0) {
                    result.normal.x = -normal.x;
                    result.normal.y = -normal.y;
                }
            }
        }
    }
    return result;
}
//calculates the normal vector from an edge defined by two points
SDL_FPoint OBB::normalFromEdge(const SDL_FPoint& p1, const SDL_FPoint& p2) const
{
    SDL_FPoint edge{ p2.x - p1.x, p2.y - p1.y };
    SDL_FPoint normal{ -edge.y, edge.x };
    float len = sqrt(normal.x * normal.x + normal.y * normal.y);
    if (len > 0.0f) {
        normal.x = normal.x / len;
        normal.y = normal.y / len;
    }
    return normal;
}
//projects the the corners of the OBB onto a given normal and finds the min and max values
void OBB::projectNormal(const std::array<SDL_FPoint, 4>& pts, const SDL_FPoint& normal, float& min, float& max) const
{
    min = max = pts[0].x * normal.x + pts[0].y * normal.y;
    for (int i = 1; i < 4; i++) {
        float proj = pts[i].x * normal.x + pts[i].y * normal.y;
        if (proj < min) {
            min = proj;
        }
        if (proj > max) {
            max = proj;
        }
    }
}
