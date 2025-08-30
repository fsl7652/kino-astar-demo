#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>
#include "OBB.h"
#include "render_circle.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cmath>
#define _USE_MATH_DEFINES

/// @brief State structure representing the car's position and heading
/// @param x X position
/// @param y Y position
/// @param yaw Heading angle in radians
struct State {
    double x, y, yaw;

    State(double x = 0, double y = 0, double yaw = 0) : x(x), y(y), yaw(yaw) {}

    bool operator==(const State& other) const {
        return std::abs(x - other.x) < 1e-3 &&
            std::abs(y - other.y) < 1e-3 &&
            std::abs(yaw - other.yaw) < 1e-3;
    }
};
/// @brief Hash function for State to use in unordered containers
/// @note Combines hashes of x, y, and yaw using XOR
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
/// @brief Node structure for A* search
/// @param state The state represented by this node
/// @param cost Cost from start to this node
/// @param heuristic Heuristic cost estimate to goal
/// @param parent Pointer to parent node for path reconstruction
/// @param motion_path Sequence of states from parent to this node

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

/// @brief Metrics for search performance tracking
/// @param status Current status of the search
/// @param path_cost Cost of the found path
/// @param path_length Length of the found path in nodes
/// @param nodes_expanded Number of nodes expanded during search
/// @param nodes_generated Number of nodes generated during search
/// @param max_open_size Maximum size of the open set during search
/// @param calc_time Total computation time in seconds
/// @param path_found Whether a valid path was found
/// @param current_open_size Current size of the open set  
/// @param current_closed_size Current size of the closed set
/// @note Provides a reset function to clear metrics
/// @note Provides a printMetrics function to output metrics to console
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
    void printMetrics() {
    std::cout << "\n=== Pathfinding Results ===" << std::endl;
    std::cout << "Status: " << status << std::endl;
    std::cout << "Time: " << calc_time << " ms" << std::endl;
    std::cout << "Nodes expanded: " << nodes_expanded << std::endl;
    std::cout << "Max open set size: " << max_open_size << std::endl;
    if (path_found) {
        std::cout << "Path length: " << path_length << " nodes" << std::endl;
        std::cout << "Path cost: " << path_cost << std::endl;
    }
    std::cout << "==========================" << std::endl;
}
};

/// @brief Kinodynamic A* path planner for a simple car model
/// @param car_length Length of the car for collision checking
/// @param car_width Width of the car for collision checking
/// @param wheelbase Distance between front and rear axles
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
