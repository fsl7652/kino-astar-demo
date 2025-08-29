#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>
#include <cmath>
#include "OBB.h"
#include "render_circle.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#define _USE_MATH_DEFINES

struct State {
    double x, y, yaw;

    State(double x = 0, double y = 0, double yaw = 0) : x(x), y(y), yaw(yaw) {}

    bool operator==(const State& other) const {
        return std::abs(x - other.x) < 1e-3 &&
            std::abs(y - other.y) < 1e-3 &&
            std::abs(yaw - other.yaw) < 1e-3;
    }
};

namespace std {
    template<>
    struct hash<State> {
        size_t operator()(const State& s) const {
            return hash<double>()(s.x) ^ hash<double>()(s.y) ^ hash<double>()(s.yaw);
        }
    };
}

struct Node;

struct NodeDeleter {
    void operator()(Node* ptr) const;
};

struct Node {
    State state;
    double cost;
    double heuristic;
    Node* parent;
    std::vector<State> motion_path;

    Node() : cost(0.0), heuristic(0.0), parent(nullptr) {}
    Node(State s, double c, double h, Node* p = nullptr)
        : state(s), cost(c), heuristic(h), parent(std::move(p)) {}

    double f_cost() const { return cost + heuristic; }
};

using NodePtr = std::unique_ptr<Node, NodeDeleter>;

struct SearchMetrics {
    std::string status = "Idle";
    double path_cost = 0.0;
    int path_length = 0;
    int nodes_expanded = 0;        
    int nodes_generated = 0;        
    int max_open_size = 0;
    double calc_time = 0.0;
    bool path_found = false;

    int current_open_size = 0;
    int current_closed_size = 0;

    void reset() {
        status = "Idle";
        path_cost = 0.0;
        path_length = 0;
        nodes_expanded = 0;
        nodes_generated = 0;
        max_open_size = 0;
        calc_time = 0.0;
        path_found = false;
    }
};

class SimpleKino {
public:
    SimpleKino(double car_length = 0.25, double car_width = 0.19, double wheelbase = 0.15);

    std::vector<State> findPath(const State& start, const OBB& goal_box, const std::vector<OBB>& obstacles);

    const std::vector<State>& getExpandedNodes() const { return expanded_nodes; }
    const std::vector<State>& getCurrentPath() const { return cur_path; }
    const bool getDisplay() const { return !cur_path.empty(); };
    const SearchMetrics& getMetrics() const { return metrics; }

    void render(SDL_Renderer* renderer, float scale = 100.0f) const;

    std::vector<State> BSplineSmooth(std::vector<State>& path, int resolution);

    void clearNodePool();

private:
    bool isStateValid(const State& state, const std::vector<OBB>& obstacles) const;
    bool isGoalReached(const State& state, const OBB& goal_box) const;
    double calculateHeuristic(const State& a, const State& b) const;
    State propagateState(const State& state, double steering, double distance) const;
    std::vector<State> replayPath(Node* goal_node) const;
    Node* allocateNode(const State& state, double cost, double heuristic, Node* parent = nullptr);

    double car_length, car_width, wheelbase;
    std::vector<State> expanded_nodes;
    std::vector<State> cur_path;

    std::vector<std::unique_ptr<Node>> node_pool;
    int next_node_index = 0;
    static const int MAX_NODES = 10000;

    SearchMetrics metrics;
};
