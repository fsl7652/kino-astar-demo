#pragma once
#define SDL_MAIN_HANDLED
#define SDL_DISABLE_IMPLICIT_LINKING
#include <SDL3/SDL.h>
#include <corecrt_math_defines.h>


class RenderCircle {
private:
    static int roundUpToMultipleOfEight(int v)
    {
        return (v + (8 - 1)) & -8;
    }
public:
    /*static inline void drawCircle(SDL_Renderer* renderer, SDL_FPoint center, int radius)
    {
        std::vector<SDL_FPoint> points;
        const int arrSize = roundUpToMultipleOfEight(radius * 8 * 35 / 49);
        points.reserve(arrSize);

        const float diameter = (radius * 2);

        float x = (radius - 1);
        float y = 0.0f;
        float tx = 1.0f;
        float ty = 1.0f;
        float error = (tx - diameter);

        while (x >= y)
        {
            points.push_back({ center.x + x, center.y - y });
            points.push_back({ center.x + x, center.y + y });
            points.push_back({ center.x - x, center.y - y });
            points.push_back({ center.x - x, center.y + y });
            points.push_back({ center.x + y, center.y - x });
            points.push_back({ center.x + y, center.y + x });
            points.push_back({ center.x - y, center.y - x });
            points.push_back({ center.x - y, center.y + x });

            if (error <= 0)
            {
                ++y;
                error += ty;
                ty += 2;
            }

            if (error > 0)
            {
                --x;
                tx += 2;
                error += (tx - diameter);
            }
        }
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        for (auto point : points) {
            SDL_RenderPoint(renderer, point.x, point.y);
        }
    }*/

    static inline void drawCircle(SDL_Renderer* renderer, SDL_FPoint center, int radius, SDL_Color colour = {255, 0, 0, 255}) {
        SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);

        const int segments = 20; // Lower for performance, higher for smoothness
        SDL_FPoint points[segments + 1];

        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            points[i].x = center.x + radius * cos(angle);
            points[i].y = center.y + radius * sin(angle);
        }

        SDL_RenderLines(renderer, points, segments + 1);
    }

    static inline void drawCircleFast(SDL_Renderer* renderer, SDL_FPoint center, int radius) {
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

        int x = radius;
        int y = 0;
        int err = 0;

        while (x >= y) {
            SDL_RenderPoint(renderer, center.x + x, center.y + y);
            SDL_RenderPoint(renderer, center.x + y, center.y + x);
            SDL_RenderPoint(renderer, center.x - y, center.y + x);
            SDL_RenderPoint(renderer, center.x - x, center.y + y);
            SDL_RenderPoint(renderer, center.x - x, center.y - y);
            SDL_RenderPoint(renderer, center.x - y, center.y - x);
            SDL_RenderPoint(renderer, center.x + y, center.y - x);
            SDL_RenderPoint(renderer, center.x + x, center.y - y);

            y += 1;
            err += 1 + 2 * y;
            if (2 * (err - x) + 1 > 0) {
                x -= 1;
                err += 1 - 2 * x;
            }
        }
    }
};