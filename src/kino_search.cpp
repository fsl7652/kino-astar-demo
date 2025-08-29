#include "kino_search.h"
#include <algorithm>
#include <cmath>
#define _USE_MATH_DEFINES

void NodeDeleter::operator()(Node* ptr) const {
    delete ptr;
}

SimpleKino::SimpleKino(double car_length, double car_width, double wheelbase)
    : car_length(car_length), car_width(car_width), wheelbase(wheelbase) {
    node_pool.resize(MAX_NODES);
    for (int i = 0; i < MAX_NODES; i++) {
        node_pool[i] = std::make_unique<Node>();
    }
    next_node_index = 0;
    metrics.reset();
}

Node* SimpleKino::allocateNode(const State& state, double cost, double heuristic, Node* parent) {
    if (next_node_index >= MAX_NODES) {
        std::cout << "ERROR: Node pool exhausted!" << std::endl;
        return nullptr;
    }

    Node* node = node_pool[next_node_index++].get();
    node->state = state;
    node->cost = cost;
    node->heuristic = heuristic;
    node->parent = parent;

    metrics.nodes_generated++;

    return node;
}

void SimpleKino::clearNodePool() {
    next_node_index = 0;
    metrics.reset();
}

std::vector<State> SimpleKino::findPath(const State& start, const OBB& goal_box, const std::vector<OBB>& obstacles) {
    auto goal = State(double(goal_box.center.x), double(goal_box.center.y), double(goal_box.angle));

    clearNodePool();
    expanded_nodes.clear();
    cur_path.clear();

    auto start_time = std::chrono::high_resolution_clock::now();
    metrics.status = "Searching...";

    Node* start_node = allocateNode(start, 0.0, calculateHeuristic(start, goal));
    if (!start_node) {
        metrics.status = "Failed: No start node";
        return {};
    }

    auto node_cmp = [](Node* a, Node* b) {
        return a->f_cost() > b->f_cost();
        };

    std::priority_queue<Node*, std::vector<Node*>, decltype(node_cmp)> open_set(node_cmp);
    std::unordered_set<State> closed_set; 

    open_set.push(start_node);
    metrics.current_open_size = 1;
    metrics.max_open_size = 1;

    while (!open_set.empty()) {
        Node* cur_node = open_set.top();
        open_set.pop();

        metrics.current_closed_size = closed_set.size();
        metrics.current_open_size = open_set.size();

        expanded_nodes.push_back(cur_node->state);
        metrics.nodes_expanded++;

        if (isGoalReached(cur_node->state, goal_box)) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            metrics.calc_time = duration.count();
            cur_path = replayPath(cur_node); 
            metrics.path_length = cur_path.size();
            metrics.path_found = true;
            metrics.path_cost = 0.0;
            metrics.status = "Path Found!";
            Node* path_node = cur_node;
            while (path_node && path_node->parent) {
                double dx = path_node->state.x - path_node->parent->state.x;
                double dy = path_node->state.y - path_node->parent->state.y;
                metrics.path_cost += std::sqrt(dx * dx + dy * dy);
                path_node = path_node->parent;
            }
            cur_path = BSplineSmooth(cur_path, 2);

            return cur_path;
        }

        if (closed_set.find(cur_node->state) != closed_set.end()) {
            continue;
        }
        closed_set.insert(cur_node->state);

        const std::vector<double> steering_angles = {-0.45, -0.3, -0.15, 0.0, 0.15, 0.3 , 0.45};
        const std::vector<double> distances = { 0.3, 0.6, 1.0 }; 

        for (double steer : steering_angles) {
            for (double dist : distances) {
                State new_state = propagateState(cur_node->state, steer, dist);

                if (!isStateValid(new_state, obstacles) ||
                    closed_set.find(new_state) != closed_set.end()) {
                    continue;
                }

                double new_cost = cur_node->cost + dist * (1.5 * std::abs(steer));
                double new_heu = calculateHeuristic(new_state, goal);

                Node* new_node = allocateNode(new_state, new_cost, new_heu, cur_node);
                if (!new_node) {
                    continue;
                }

                open_set.push(new_node);
                metrics.current_open_size = open_set.size();
                if (open_set.size() > metrics.max_open_size) {
                    metrics.max_open_size = open_set.size();
                }
            }
        }
    }

    if (!metrics.path_found) {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        metrics.calc_time = duration.count();
        metrics.status = "No Path Found";
    }

    std::cout << "\n=== Pathfinding Results ===" << std::endl;
    std::cout << "Status: " << metrics.status << std::endl;
    std::cout << "Time: " << metrics.calc_time << " ms" << std::endl;
    std::cout << "Nodes expanded: " << metrics.nodes_expanded << std::endl;
    std::cout << "Max open set size: " << metrics.max_open_size << std::endl;

    if (metrics.path_found) {
        std::cout << "Path length: " << metrics.path_length << " nodes" << std::endl;
        std::cout << "Path cost: " << metrics.path_cost << std::endl;
    }
    std::cout << "==========================" << std::endl;
    return {};
}

std::vector<State> SimpleKino::replayPath(Node* goal_node) const {
    std::vector<State> path;

    Node* cur_node = goal_node;
    while (cur_node != nullptr) {
        path.push_back(cur_node->state);
        cur_node = cur_node->parent; 
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool SimpleKino::isStateValid(const State& state, const std::vector<OBB>& obstacles) const {
    OBB car_box(state.x, state.y, car_length, car_width, state.yaw);

    for (const auto& obs : obstacles) {
        CollisionResult res = car_box.expandedCollision(obs);
        if (res.collided) {
            return false;
        }
    }
    return true;
}

bool SimpleKino::isGoalReached(const State& state, const OBB& goal_box) const {
    OBB car_box(state.x, state.y, car_length, car_width, state.yaw);
    OBB small_goal(goal_box.center.x, goal_box.center.y, 0.1f, 0.1f);
    CollisionResult res = car_box.collision(small_goal);
    return res.collided;
}

double SimpleKino::calculateHeuristic(const State& a, const State& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double yaw_diff = std::abs(std::fmod(a.yaw - b.yaw + M_PI, 2.0 * M_PI) - M_PI);

    return distance + 0.3 * yaw_diff;
}

State SimpleKino::propagateState(const State& state, double steering, double distance) const {
    State new_state = state;

    if (std::abs(steering) < 1e-3) {
        new_state.x += distance * std::cos(state.yaw);
        new_state.y += distance * std::sin(state.yaw);
    }
    else {
        double turning_radius = wheelbase / std::tan(steering);
        double angular_change = distance / turning_radius;

        new_state.x += turning_radius * (std::sin(state.yaw + angular_change) - std::sin(state.yaw));
        new_state.y -= turning_radius * (std::cos(state.yaw + angular_change) - std::cos(state.yaw));
        new_state.yaw = std::fmod(state.yaw + angular_change, 2.0 * M_PI);
    }
    return new_state;
}


void SimpleKino::render(SDL_Renderer* renderer, float scale) const {
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 100);
    for (const auto& node : expanded_nodes) {
        SDL_FPoint pt = { float(node.x * scale), float(node.y * scale) };
        RenderCircle::drawCircle(renderer, pt, 2);
    }

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    for (size_t i = 0; i < cur_path.size() - 1; ++i) {
        SDL_RenderLine(renderer,
            cur_path[i].x * scale, cur_path[i].y * scale,
            cur_path[i + 1].x * scale, cur_path[i + 1].y * scale);
    }


}

std::vector<State> SimpleKino::BSplineSmooth(std::vector<State>& path, int resolution) {
    if (path.size() < 4) {
        return path;
    }

    std::vector<State> smooth_path;
    int count = static_cast<int>(path.size()) - 1;

    for (int i = 0; i <= count * resolution; i++) {
        float t = static_cast<float>(i) / resolution;
        int seg_num = static_cast<int>(t);
        t -= seg_num;
        seg_num = std::min(seg_num, count - 3); 

        float t2 = t * t;
        float t3 = t2 * t;

        float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
        float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
        float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
        float b3 = t3 / 6.0f;

        State point;
        point.x = b0 * path[seg_num].x +
            b1 * path[seg_num + 1].x +
            b2 * path[seg_num + 2].x +
            b3 * path[seg_num + 3].x;

        point.y = b0 * path[seg_num].y +
            b1 * path[seg_num + 1].y +
            b2 * path[seg_num + 2].y +
            b3 * path[seg_num + 3].y;

        if (!smooth_path.empty()) {
            float dx = point.x - smooth_path.back().x;
            float dy = point.y - smooth_path.back().y;
            point.yaw = std::atan2(dy, dx);
        }
        else {
            point.yaw = path.front().yaw;
        }

        smooth_path.push_back(point);
    }

    smooth_path.push_back(path.back());

    return smooth_path;
}
