#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <chrono>

#include "Environment.h"
#include "Plane.h"
#include "State.h"
#include "ControlSurfaces.h"
#include "Obstacle3D.h"
#include "Dynamics.h"
#include "Graph.h"
#include "Witness.h"

void exportBestPathToCSV(const std::vector<std::shared_ptr<Node>>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open the file for writing the best path.");
    }

    // Write header
    file << "NodeID,x,y,z,phi,theta,psi,u,v,w,p,q,r,delta_e,delta_a,delta_r,duration,cost\n";

    int node_id = 0;
    for (const auto& node : path) {
        const State& s = node->state;
        const ControlSurfaces& c = node->control;
        file << node_id << ","
             << s.x << "," << s.y << "," << s.z << ","
             << s.phi << "," << s.theta << "," << s.psi << ","
             << s.u << "," << s.v << "," << s.w << ","
             << s.p << "," << s.q << "," << s.r << ","
             << c.delta_e << "," << c.delta_a << "," << c.delta_r << ","
             << node->duration << ","
             << node->cost << "\n";
        ++node_id;
    }

    file.close();
}

std::vector<std::shared_ptr<Node>> extractBestPath(const std::shared_ptr<Node>& end_node) {
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> current_node = end_node;

    while (current_node != nullptr) {
        path.push_back(current_node);
        current_node = current_node->parent;
    }

    // Reverse the path to get it from start to end
    std::reverse(path.begin(), path.end());
    return path;
}

// Check if a node is a leaf
bool isLeaf(const std::shared_ptr<Node>& node) {
    return node->children.empty();
}

// Check if a node is in a given set
bool isInSet(const std::shared_ptr<Node>& node, const std::vector<std::shared_ptr<Node>>& set) {
    return std::find(set.begin(), set.end(), node) != set.end();
}

void pruneDominatedNodes(
    const std::shared_ptr<Node>& x_new,
    std::vector<std::shared_ptr<Node>>& Vactive,
    std::vector<std::shared_ptr<Node>>& Vinactive,
    Graph& graph,
    WitnessSet& witness_set,
    const std::shared_ptr<Node>& x0_node
) {
    // Find the nearest witness to x_new
    Witness* s_new = witness_set.nearestWitness(x_new->state);

    // Get the representative of the nearest witness
    std::shared_ptr<Node> x_peer = s_new->rep;

    // If x_peer exists and is not the root
    if (x_peer && x_peer != x0_node) {
        // Move x_peer to Vinactive
        auto it = std::find(Vactive.begin(), Vactive.end(), x_peer);
        if (it != Vactive.end()) {
            Vactive.erase(it);
            Vinactive.push_back(x_peer);
            x_peer->active = false; // Mark as inactive
        }
        s_new->rep = x_new; // Update the representative
    } else {
        s_new->rep = x_new; // Update the representative
    }

    // Begin pruning dominated nodes
    while (x_peer && isLeaf(x_peer) && isInSet(x_peer, Vinactive) && x_peer != x0_node) {
        auto x_parent = x_peer->parent;
        if (!x_parent) {
            break;
        }

        // Remove edges
        graph.disconnectNodes(x_parent, x_peer);

        // Remove x_peer from Vinactive
        Vinactive.erase(std::remove(Vinactive.begin(), Vinactive.end(), x_peer), Vinactive.end());

        // Check if parent is also a leaf and in Vinactive
        if (isLeaf(x_parent) && isInSet(x_parent, Vinactive)) {
            x_peer = x_parent; // Move up the hierarchy
        } else {
            break;
        }
    }
}

void runSST(int N, double deltaBN, double deltaS,
            std::shared_ptr<Node>& best_node,
            Graph& graph,
            std::vector<std::shared_ptr<Node>>& Vactive,
            std::vector<std::shared_ptr<Node>>& Vinactive,
            WitnessSet& witness_set,
            const std::shared_ptr<Node>& x0_node,
            Environment& env) {

    Dynamics dynamics;
    bool goal_found = false;

    for (int iter = 0; iter < N; ++iter) {
        // Sample a random state
        State s_sample = env.sampleRandomState();

        // Find the nearest neighbor in the active set
        auto x_nearest_node = graph.nearestNeighbor(s_sample, true);
        if (!x_nearest_node) {
            continue;
        }

        State x_nearest_state = x_nearest_node->state;

        // Sample multiple controls
        const int num_controls = 20;
        std::vector<ControlSurfaces> control_samples;
        std::vector<State> state_samples;
        std::vector<double> duration_samples;

        for (int i = 0; i < num_controls; ++i) {
            ControlSurfaces control_surfaces_new = env.sampleRandomControls();
            double duration_new = env.sampleRandomTimeStep();

            // Propagate the state
            State x_new_state = dynamics.getNextState(env, x_nearest_state, control_surfaces_new, duration_new);

            // Check collision
            if (!env.isPlaneInCollision(x_nearest_state, x_new_state)) {
                control_samples.push_back(control_surfaces_new);
                state_samples.push_back(x_new_state);
                duration_samples.push_back(duration_new);
            }
        }

        // If no valid samples, continue
        if (state_samples.empty()) {
            continue;
        }

        // Select the control based on cost minimization or range
        size_t best_index = 0;
        if (env.sample_goal_state) {
            double min_cost = env.costFunction(state_samples[0], control_samples[0], duration_samples[0]);
            for (size_t i = 1; i < state_samples.size(); ++i) {
                double cost_i = env.costFunction(state_samples[i], control_samples[i], duration_samples[i]);
                if (cost_i < min_cost) {
                    min_cost = cost_i;
                    best_index = i;
                }
            }
        } else {
            double min_cost = env.costFunction(state_samples[0], control_samples[0], duration_samples[0]);
            for (size_t i = 1; i < state_samples.size(); ++i) {
                double cost_i = env.costFunction(state_samples[i], control_samples[i], duration_samples[i]);
                if (cost_i < min_cost) {
                    min_cost = cost_i;
                    best_index = i;
                }
            }
        }

        // Use the best control and state
        ControlSurfaces best_control = control_samples[best_index];
        State best_state = state_samples[best_index];
        double best_duration = duration_samples[best_index];
        double cost_new = env.costFunction(best_state, best_control, best_duration);

        // Create the new node
        auto x_new_node = std::make_shared<Node>(Node{best_state, best_control, best_duration, cost_new, false});

        // Check if the new node is locally the best
        if (witness_set.isNodeLocallyBest(x_new_node, deltaS)) {
            x_new_node->active = true;
            Vactive.push_back(x_new_node);
            graph.addNode(x_new_node);
            graph.connectNodes(x_nearest_node, x_new_node);

            // Update the best node if cost is improved
            if (x_new_node->cost < best_node->cost) {
                best_node = x_new_node;
            }

            // Check goal conditions
            if (env.sample_goal_state && env.isGoalReached(x_new_node->state)) {
                best_node = x_new_node;
                goal_found = true;
                break;
            }

            if (!env.sample_goal_state && std::abs(x_new_node->state.x - env.x_max) < 50) {
                std::cout << "Maximum x reached at iteration " << iter << "\n";
                best_node = x_new_node;
                goal_found = true;
                break;
            }

            // Prune dominated nodes
            pruneDominatedNodes(x_new_node, Vactive, Vinactive, graph, witness_set, x0_node);
        }

        // Progress feedback
        if (iter % 10000 == 0 && iter != 0) {
            std::cout << "SST Iteration: " << iter
                      << ", Best Node X: " << best_node->state.x << "\n";
        }
    }

    if (goal_found) {
        std::cout << "\033[32mGoal has been reached within the runSST function.\033[0m\n";
    } else {
        std::cout << "\033[31mGoal has not been reached within the runSST function.\033[0m\n";
    }
}

bool runSSTStar() {
    // Algorithm 9 parameters
    const double xi = 0.3;
    const int d = 12;
    const int l = 3;
    const int N0 = 500;
    const double deltaBN0 = 30.0;
    const double deltaS0 = 15.0;
    const int max_outer_iterations = 30;

    int j = 0;
    int N = N0;
    double deltaS = deltaS0;
    double deltaBN = deltaBN0;

    Dynamics dynamics;
    Environment env;
    State x0 = env.initial_state;
    ControlSurfaces u0 = env.initial_control;
    double c0 = env.costFunction(x0, u0, 0.0);

    Graph graph;
    graph.setDeltaBN(deltaBN);

    std::vector<std::shared_ptr<Node>> Vactive;
    std::vector<std::shared_ptr<Node>> Vinactive;

    auto x0_node = graph.addNode(x0, u0, 0.0, c0, true);
    Vactive.push_back(x0_node);

    WitnessSet witness_set;
    Witness s0(x0);
    s0.rep = x0_node;
    s0.cost = c0;
    witness_set.addWitness(s0);

    std::shared_ptr<Node> best_node = x0_node;

    while (j < max_outer_iterations) {
        graph.setDeltaBN(deltaBN);
        runSST(N, deltaBN, deltaS, best_node, graph, Vactive, Vinactive, witness_set, x0_node, env);

        deltaS *= xi;
        deltaBN *= xi;
        j += 1;

        double log_j = (j > 0) ? std::log(static_cast<double>(j)) : 0.0;
        double exponent = -(d + l + 1) * j;
        double xi_power = std::pow(xi, exponent);
        double N_double = (1.0 + log_j) * xi_power * N0;
        N = static_cast<int>(N_double);
        if (N < 1000) {
            N = 1000;
        }
        if (N > 100000) {
            N = 100000;
        }
    }

    if (env.isGoalReached(best_node->state)) {
        std::cout << "\033[32mGoal has been reached within the runSSTStar function.\033[0m\n";
        std::vector<std::shared_ptr<Node>> best_path = extractBestPath(best_node);
        exportBestPathToCSV(best_path, "best_path_sststar.csv");
        return true;
    }

    return false;
}

int main() {
    std::cout << "Select the algorithm to run:\n";
    std::cout << "1. SST (Stable Sparse RRT)\n";
    std::cout << "2. SST* (Stable Sparse RRT*)\n";
    std::cout << "Enter your choice (1 or 2): ";

    int choice;
    // std::cin >> choice;
    choice = 2; // Hard-coded choice for convenience

    if (choice == 1) {
        std::cout << "Running SST...\n";

        const int N_SST = 1000;      
        const double deltaBN_SST = 30.0;
        const double deltaS_SST = 15.0;

        Dynamics dynamics;
        Environment env;
        State x0 = env.initial_state;
        ControlSurfaces u0 = env.initial_control;
        double dt0 = 0.0;
        double c0 = env.costFunction(x0, u0, dt0);

        Graph graph;
        std::vector<std::shared_ptr<Node>> Vactive;
        std::vector<std::shared_ptr<Node>> Vinactive;

        auto x0_node = graph.addNode(x0, u0, dt0, c0, true);
        Vactive.push_back(x0_node);

        WitnessSet witness_set;
        Witness s0(x0);
        s0.rep = x0_node;
        s0.cost = c0;
        witness_set.addWitness(s0);

        std::shared_ptr<Node> best_node = x0_node;

        auto start = std::chrono::high_resolution_clock::now();
        for (int iter = 0; iter < 1; ++iter) {
            runSST(N_SST, deltaBN_SST, deltaS_SST, best_node, graph, Vactive, Vinactive, witness_set, x0_node, env);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "runSST() took " << elapsed.count() << " seconds." << std::endl;

        if (env.isGoalReached(best_node->state)) {
            std::cout << "Goal reached in SST.\n";
        }

        std::vector<std::shared_ptr<Node>> best_path = extractBestPath(best_node);
        exportBestPathToCSV(best_path, "best_path_sst.csv");
        graph.exportToCSV("tree_sst.csv");
        witness_set.exportToCSV("witness_set_sst.csv");

        std::cout << "SST completed.\n";
        std::cout << "Best path cost: " << best_node->cost << "\n";
        std::cout << "Reached Destination: (" << best_node->state.x << ", " 
                  << best_node->state.y << ", " << best_node->state.z << ")\n";
    } else if (choice == 2) {
        double total_time = 0.0;
        double M = 10000.0;
        bool passed = false;

        for (int i = 0; i < M; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            passed = runSSTStar();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            total_time += elapsed.count();
            if (passed) {
                break;
            }
        }

        double average_time = total_time / M;
        // std::cout << "Average time: " << average_time << " seconds." << std::endl;

    } else {
        std::cout << "Invalid choice. Please run the program again and select either 1 or 2.\n";
    }

    return 0;
}
