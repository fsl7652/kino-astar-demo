#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include "jetracer.h"
#include "OBB.h"
#include "wall.h"
#include <vector>
#include "goal.h"
#include "kino_search.h"
#include "render_circle.h"

static SDL_Window* window = NULL;
static SDL_Renderer* renderer = NULL;


struct Map {
    std::string name;
    std::vector<Wall> walls;
    JetRacer car_start;
};

struct AppState {
    JetRacer car;
    std::vector<Wall> walls;
    std::vector<Wall> bounds;
    Goal goal;
    SimpleKino path_planner;
    AppState() : car(0.0f, 0.0f, 0.0f) {}
    std::vector<OBB> obstacles;
    bool obs_changed;
    bool obs_placing = false;
    SDL_FPoint obs_start;
    SDL_FPoint mouse;
    float m_wheel;
    float obs_angle;
    int current_map = 0;
    std::vector<Map> maps;
    bool map_changed = false;
};

void addBounds(void* appstate) {

    AppState* state = (AppState*)appstate;
    state->bounds.emplace_back(0.0f, 0.0f, 38.4f, 0.1f);
    state->bounds.emplace_back(0.0f, 10.8f, 38.4f, 0.1f);
    state->bounds.emplace_back(19.2f, 0.0f, 0.1f, 21.6f);
    state->bounds.emplace_back(0.0f, 0.0f, 0.1f, 21.6f);

    for (auto bound : state->bounds)
    {
        state->obstacles.emplace_back(bound.box);
    }
}


void initMaps(void* appstate) {
    AppState* state = (AppState*)appstate;
    Map default_map;
    default_map.name = "Default Map";
    default_map.walls = { Wall(3.0f, 2.0f, 1.0f, 2.0f), Wall(5.0f, 4.0f, 3.0f, 0.5f), Wall(6.0f, 2.0f, 2.0f, 1.0f, 0.3f) };
    default_map.car_start = JetRacer(2.0f, 2.0f, 0.0f);
    state->maps.push_back(default_map);

    Map maze_map;
    maze_map.name = "Maze Map";
    maze_map.walls = {

       Wall(3.0f, 2.4f, 0.5f, 4.0f, 0.0f),    
    Wall(16.2f, 5.4f, 0.5f, 4.0f, 0.0f),     
    Wall(3.0f, 8.4f, 0.5f, 4.0f, 0.0f),    

    Wall(9.6f, 3.8f, 6.0f, 0.5f, 0.0f),
    Wall(9.6f, 7.0f, 6.0f, 0.5f, 0.0f),

    Wall(12.8f, 2.2f, 3.5f, 0.5f,  M_PI / 36),  
    Wall(6.4f, 8.8f, 3.5f, 0.5f,  -M_PI / 36) 
    };
    maze_map.car_start = JetRacer(1.0f, 1.0f, 0.0f);
    state->maps.push_back(maze_map);

    Map track_map;
    track_map.name = "Race Track Map";
    track_map.walls = {
        Wall(12.7f, 2.2f, 6.2f, 0.5f,  M_PI / 36), // top oval +5 deg
        Wall(6.5f, 2.2f, 6.2f, 0.5f, -M_PI / 36), //top oval -5 deg
    Wall(3.2f, 5.4f, 0.5f, 6.4f,   0.0f),   // left wall 
    Wall(12.7f, 8.6f, 6.2f, 0.5f, -M_PI / 36), //bottom oval -5 deg
    Wall(6.5f, 8.6f, 6.2f, 0.5f, M_PI / 36), //bottom oval +5 deg
    Wall(16.0f, 5.4f, 0.5f, 6.4f,  0.0f),     //right wall

    Wall(5.0f, 5.4f, 1.2f, 0.5f,  M_PI / 12), //obstacle 1
    Wall(14.2f, 5.4f, 1.2f, 0.5f, -M_PI / 12), // obstacle 2

    Wall(9.6, 5.4f, 4.0f, 2.0f) //center rect
    };

    track_map.car_start = JetRacer(5.0f, 3.0f, 0.0f);
    state->maps.push_back(track_map);
}

void loadMap(void* appstate, int map_idx) {
    AppState* state = (AppState*)appstate;
    if (map_idx < 0 || map_idx >= state->maps.size())
    {
        return;
    }
    state->current_map = map_idx;
    Map& map = state->maps[map_idx];

    state->walls.clear();
    state->obstacles.clear();

    state->walls = map.walls;
    state->car = map.car_start;

    for (const auto& wall : state->walls) {
        state->obstacles.emplace_back(wall.box);
    }
    addBounds(appstate);
    state->map_changed = true;
    state->obs_changed = true;
}



void mousePress(void* appstate, SDL_MouseButtonEvent& b) {
    AppState* state = (AppState*)appstate;
    const float scale = 100.0f;
    float worldX = b.x / scale;
    float worldY = b.y / scale;
    if (b.button == SDL_BUTTON_LEFT) {
        
        std::vector<OBB> wall_boxes;

        Goal check_goal = Goal(worldX, worldY, 0.25f, 0.25f, 0.0f, true);
        bool valid = true;

        OBB goal_box = check_goal.box;
        for (const auto& obs : state->obstacles) {
            CollisionResult valid_res = goal_box.collision(obs);
            if (valid_res.collided) {
                SDL_Log("Invalid Goal, within object");
                valid = false;
            }
        }
        if (valid) {
            state->goal = check_goal;
            state->goal.display = true;
            SDL_Log("Valid Goal");

            State car_state(state->car.x, state->car.y, state->car.yaw);
            state->path_planner.findPath(car_state, state->goal.box, state->obstacles);

            const std::vector<State>& new_path = state->path_planner.getCurrentPath();
            if (!new_path.empty())
                state->car.setPath(new_path);
        }
    }
    else if (b.button == SDL_BUTTON_RIGHT)
    {
        if (!state->obs_placing)
        {
            state->obs_start = { worldX, worldY };
            state->obs_placing = true;
        }
        else {
            SDL_FPoint obs_finish{ worldX, worldY };
            float screen_w = state->obs_start.x - state->mouse.x;
            float screen_h = state->obs_start.y - state->mouse.y;

            float world_w = std::abs(screen_w);
            float world_h = std::abs(screen_h);
            float center_x = (std::min(state->obs_start.x, state->mouse.x) + std::abs(screen_w) / 2);
            float center_y = (std::min(state->obs_start.y, state->mouse.y) + std::abs(screen_h) / 2);

            bool valid_place = true;

            OBB check_obs = OBB(center_x, center_y, world_w, world_h, state->obs_angle);

            for (auto& obs: state->obstacles)
            {
                CollisionResult valid_obs = check_obs.collision(obs);
                if (valid_obs.collided)
                {
                    valid_place = false;
                }
            }
            if (check_obs.collision(state->car.getOBB()).collided)
            {
                valid_place = false;
            }
            if (check_obs.collision(state->goal.box).collided)
            {
                valid_place = false;
                
            }

            if (valid_place) {
                Wall new_obs = Wall(center_x, center_y, world_w, world_h, state->obs_angle);
                state->walls.emplace_back(new_obs);
                state->obstacles.emplace_back(new_obs.box);
                state->obs_changed = true;
            }
            state->obs_placing = false;
        }
    }
}

void showHelp() {
    std::cout << "JetRacer Pathfinding Demo\n";
    std::cout << "Usage: JetRacerDemo [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --help     Show this help message\n";
    std::cout << "  --version  Show version information\n";
    std::cout << "  --test     Run in test mode (exit after initialization)\n";
}



SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[])
{
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) {
            showHelp();
            return SDL_APP_SUCCESS;
        }
        else if (strcmp(argv[i], "--version") == 0) {
            std::cout << "JetRacerDemo version 1.0.0\n";
            return SDL_APP_SUCCESS;
        }
        else if (strcmp(argv[i], "--test") == 0) {
            std::cout << "Test mode: Initialization successful\n";
            return SDL_APP_SUCCESS;
        }
    }

    SDL_SetAppMetadata("JetRacer Demo", "1.0", "");

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("JetRacer Demo", 1920, 1080, 0, &window, &renderer)) {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }


    AppState* state = new AppState();

    addBounds(state);

    initMaps(state);

    loadMap(state, 0);

    

    *appstate = state;
    return SDL_APP_CONTINUE;
}
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event)
{
    AppState* state = (AppState*)appstate;
    switch (event->type)
    {
    case SDL_EVENT_QUIT:
        return SDL_APP_SUCCESS;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        mousePress(appstate, event->button);
        break;
    case SDL_EVENT_MOUSE_MOTION:
        state->mouse = { event->motion.x / 100.0f, event->motion.y /100.0f }; 
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        if (event->wheel.y > 0) {
            state->obs_angle += 0.1f;
            if (state->obs_angle > M_PI) {
                state->obs_angle = 0.0f;
            }
        }
        else if (event->wheel.y < 0) {
            state->obs_angle -= 0.1f;
            if (state->obs_angle < -M_PI) {
                state->obs_angle = 0.0f;
            }
        }
    case SDL_EVENT_KEY_DOWN:
        if (event->key.key >= SDLK_1 && event->key.key <= SDLK_9)
        {
            int map_idx = event->key.key - SDLK_1;
            if (map_idx < state->maps.size()) {
                loadMap(state, map_idx);
            }
        }
        else if (event->key.key == SDLK_TAB) {
            int next_map = (state->current_map + 1) % state->maps.size();
            loadMap(state, next_map);
        }
        break;
    default:
        break;
    }
    return SDL_APP_CONTINUE;
}
SDL_AppResult SDL_AppIterate(void* appstate)
{
    AppState* state = (AppState*)appstate;
    JetRacer& car = state->car;
    std::vector<Wall>& walls = state->walls;
    Goal& goal = state->goal; 
    SimpleKino& planner = state->path_planner; 
    std::vector<Wall>& bounds = state->bounds;
    std::vector<OBB> obstacles = state->obstacles;
    float dt = 0.016f;
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    const bool* keys = SDL_GetKeyboardState(NULL);

    if (!car.getPath().empty()) {
        if (state->obs_changed) {
            if (car.pathIntersectsObstacles(obstacles)) {
                car.clearPath();
                auto new_path = planner.findPath(State(car.x, car.y, car.yaw),goal.box,obstacles);
                car.setPath(new_path);
            }
            state->obs_changed = false; 
        }
        car.purePursuitPath(car.getPath(), goal.box, dt);
    }
    else {
        float throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f) +
            (keys[SDL_SCANCODE_DOWN] ? -1.0f : 0.0f);
        float steering = (keys[SDL_SCANCODE_LEFT] ? -0.3f : 0.0f) +
            (keys[SDL_SCANCODE_RIGHT] ? 0.3f : 0.0f);
        car.update(dt, throttle, steering);
    }

    if (keys[SDL_SCANCODE_ESCAPE] > 0.0f)
    {
        return SDL_APP_SUCCESS;
    } 

    if (state->m_wheel > 0.0f) {
        if (state->obs_angle < M_PI) { state->obs_angle += 0.1f; };
    }

    

    OBB car_box = car.getOBB();
    for (const auto& obs : obstacles) {
        CollisionResult res = car_box.collision(obs);
        if (res.collided) {
            car.x += res.normal.x * res.depth;
            car.y += res.normal.y * res.depth;

            float dot = car.v.x * res.normal.x + car.v.y * res.normal.y;
            if (dot < 0) {
                float bounce = 0.8f;
                car.v.x -= (1.0f + bounce) * dot * res.normal.x;
                car.v.y -= (1.0f + bounce) * dot * res.normal.y;
            }

            car_box = car.getOBB();
        }
    }
    CollisionResult goal_res = car_box.collision(goal.box);
    if (goal_res.collided)
    {
        SDL_Log("Reached Goal!");
        state->goal = Goal(); 
        car.stop();
    }


    //RenderCircle::drawCircle(renderer, { 100.0f, 100.0f }, 20);

    for (int i = 0; i < walls.size(); i++)
    {
        walls[i].render(renderer);
    }

    for (int i = 0; i < bounds.size(); i++)
    {
        SDL_Color blank = {0,0,0,0};
        bounds[i].render(renderer, 100.0f, blank);
    }

    if (goal.display) {
        goal.render(renderer);
    }

    car.render(renderer);

    if (planner.getDisplay())
    {
        planner.render(renderer);
    }
    if (state->obs_placing) {
        float screen_w = state->obs_start.x - state->mouse.x;
        float screen_h = state->obs_start.y - state->mouse.y;

        float world_w = std::abs(screen_w);
        float world_h = std::abs(screen_h);
        float center_x = (std::min(state->obs_start.x, state->mouse.x) + std::abs(screen_w) / 2);
        float center_y = (std::min(state->obs_start.y, state->mouse.y) + std::abs(screen_h) / 2);
        Wall preview_wall = Wall(center_x, center_y, world_w, world_h, state->obs_angle);
        //SDL_Log("Mouse: %.2f, %.2f Start: %.2f, %.2f", state->mouse.x, state->mouse.y, state->obs_start.x, state->obs_start.y);
        preview_wall.render(renderer);
    }
    SDL_RenderPresent(renderer);

    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult result)
{
    delete (AppState*)appstate;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

bool AABB(const SDL_FRect& a, const SDL_FRect& b) {
    return a.x < b.x + b.w &&
        a.x + a.w > b.x &&
        a.y < b.y + b.h &&
        a.y + a.h > b.y;
}
