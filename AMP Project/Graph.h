// Graph.hpp
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "Node.h"
#include "State.h"
#include <memory>
#include <limits>
#include <unordered_map>
#include <fstream>
#include <vector>
#include <queue>
#include <tuple>
#include <cmath>       // For floor
#include <algorithm>   // For std::remove, std::find

// Define a hash function for tuples to use them as keys in unordered_map
struct TupleHash {
    std::size_t operator()(const std::tuple<int, int, int>& key) const {
        return std::hash<int>()(std::get<0>(key)) ^
               (std::hash<int>()(std::get<1>(key)) << 1) ^
               (std::hash<int>()(std::get<2>(key)) << 2);
    }
};

class Graph {
public:
    Graph() : deltaBN(5.0) {} // Default deltaBN, can be set later

    // Set a new deltaBN and re-bin all existing nodes
    void setDeltaBN(double new_deltaBN) {
        if (deltaBN == new_deltaBN) {
            return; // No change needed
        }

        deltaBN = new_deltaBN;

        // Clear the existing grid
        grid.clear();

        // Re-bin all nodes with the new deltaBN
        for (const auto& node : nodes) {
            auto bin = getBin(node->state);
            grid[bin].push_back(node);
        }
    }

    // Add a node to the graph with spatial binning
    std::shared_ptr<Node> addNode(const State& state, const ControlSurfaces& control,
                                  double duration, double cost, bool active) {
        auto node = std::make_shared<Node>(Node{state, control, duration, cost, active});
        nodes.push_back(node);

        // Compute bin indices
        auto bin = getBin(state);

        // Insert the node into the bin
        grid[bin].push_back(node);

        return node;
    }

    // Overload for adding an existing node
    void addNode(const std::shared_ptr<Node>& node) {
        nodes.push_back(node);
        auto bin = getBin(node->state);
        grid[bin].push_back(node);
    }

    // Connect two nodes (parent -> child)
    void connectNodes(const std::shared_ptr<Node>& parent, const std::shared_ptr<Node>& child) {
        parent->children.push_back(child);
        child->parent = parent; // Set the parent pointer
    }

    // Disconnect two nodes
    void disconnectNodes(const std::shared_ptr<Node>& parent, const std::shared_ptr<Node>& child) {
        parent->children.erase(std::remove(parent->children.begin(), parent->children.end(), child), parent->children.end());
        child->parent = nullptr; // Remove the parent pointer
    }

    // Remove a node from the graph
    void removeNode(const std::shared_ptr<Node>& node) {
        // Remove the node from its parent's children list
        if (node->parent) {
            auto& siblings = node->parent->children;
            siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
        }

        // Remove the node from the graph's node list
        nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());

        // Remove the node from its bin
        auto bin = getBin(node->state);
        auto& bin_nodes = grid[bin];
        bin_nodes.erase(std::remove(bin_nodes.begin(), bin_nodes.end(), node), bin_nodes.end());

        // If the bin is empty, remove it from the grid
        if (bin_nodes.empty()) {
            grid.erase(bin);
        }
    }

    // Nearest neighbor search with deltaBN
    std::shared_ptr<Node> nearestNeighbor(const State& queryState, bool onlyActive) const {
        if (nodes.empty()) {
            return nullptr; // No nodes in the graph
        }

        // Compute bin indices for the query state
        auto bin = getBin(queryState);

        // Generate a list of neighboring bins to search
        std::vector<std::tuple<int, int, int>> neighboring_bins = getNeighboringBins(bin);

        std::shared_ptr<Node> nearest = nullptr;
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& neighbor_bin : neighboring_bins) {
            auto it = grid.find(neighbor_bin);
            if (it == grid.end()) {
                continue; // No nodes in this bin
            }

            for (const auto& node : it->second) {
                if (onlyActive && !node->active) {
                    continue; // Skip inactive nodes if onlyActive is true
                }

                // Compute distance between queryState and the current node's state
                double distance = computeDistance(queryState, node->state);

                // Update nearest if this distance is smaller
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest = node;
                }
            }
        }

        return nearest; // Return the nearest node (nullptr if none found)
    }

    const std::vector<std::shared_ptr<Node>>& getNodes() const {
        return nodes;
    }

    void exportToCSV(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file for writing.");
        }

        // Write header
        file << "NodeID,ParentID,x,y,z,cost,active\n";

        // Iterate through nodes and assign IDs based on their index
        for (size_t id = 0; id < nodes.size(); ++id) {
            const auto& node = nodes[id];
            int parent_id = node->parent ? findNodeIndex(node->parent) : -1;
            file << id << "," << parent_id << ","
                 << node->state.x << "," << node->state.y << "," << node->state.z << ","
                 << node->cost << "," << node->active << "\n";
        }

        file.close();
        // std::cout << "Graph successfully exported to " << filename << "\n";
    }

    void print() const {
        if (nodes.empty()) {
            std::cout << "The graph is empty.\n";
            return;
        }

        // Assign unique IDs to nodes based on their index
        for (size_t id = 0; id < nodes.size(); ++id) {
            const auto& node = nodes[id];
            std::cout << "Node " << id << ":\n";
            std::cout << "  State: [" << node->state.x << ", " << node->state.y << ", " << node->state.z
                      << ", " << node->state.phi << ", " << node->state.theta << ", " << node->state.psi
                      << ", " << node->state.u << ", " << node->state.v << ", " << node->state.w
                      << ", " << node->state.p << ", " << node->state.q << ", " << node->state.r << "]\n";
            std::cout << "  Duration: " << node->duration << "\n";
            std::cout << "  Cost: " << node->cost << "\n";
            std::cout << "  Active: " << (node->active ? "Yes" : "No") << "\n";

            // Print parent information
            if (node->parent) {
                // Find the index of the parent node
                int parent_id = findNodeIndex(node->parent);
                std::cout << "  Parent: Node " << parent_id << "\n";
            } else {
                std::cout << "  Parent: None\n";
            }

            // Print children information
            std::cout << "  Children: ";
            if (node->children.empty()) {
                std::cout << "None";
            } else {
                for (const auto& child : node->children) {
                    int child_id = findNodeIndex(child);
                    std::cout << "Node " << child_id << " ";
                }
            }
            std::cout << "\n";
        }
    }

    void getVertexSets(std::vector<std::shared_ptr<Node>>& activeNodes, std::vector<std::shared_ptr<Node>>& inactiveNodes) const {
        activeNodes.clear();
        inactiveNodes.clear();

        for (const auto& node : nodes) {
            if (node->active) {
                activeNodes.push_back(node);
            } else {
                inactiveNodes.push_back(node);
            }
        }
    }

private:
    std::vector<std::shared_ptr<Node>> nodes;

    // Grid-based spatial partitioning
    std::unordered_map<std::tuple<int, int, int>, std::vector<std::shared_ptr<Node>>, TupleHash> grid;
    double deltaBN; // Current bin size

    // Compute the bin index for a given state
    std::tuple<int, int, int> getBin(const State& state) const {
        int bin_x = static_cast<int>(std::floor(state.x / deltaBN));
        int bin_y = static_cast<int>(std::floor(state.y / deltaBN));
        int bin_z = static_cast<int>(std::floor(state.z / deltaBN));
        return std::make_tuple(bin_x, bin_y, bin_z);
    }

    // Get neighboring bins (including the current bin)
    std::vector<std::tuple<int, int, int>> getNeighboringBins(const std::tuple<int, int, int>& bin) const {
        std::vector<std::tuple<int, int, int>> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    neighbors.emplace_back(std::make_tuple(std::get<0>(bin) + dx,
                                                          std::get<1>(bin) + dy,
                                                          std::get<2>(bin) + dz));
                }
            }
        }
        return neighbors;
    }

    // Compute Euclidean distance (you can extend this for higher dimensions)
    static double computeDistance(const State& a, const State& b) {
        return std::sqrt(
            std::pow(a.x - b.x, 2) +
            std::pow(a.y - b.y, 2) +
            std::pow(a.z - b.z, 2)
        );
    }

    // Find the index of a node in the nodes vector
    int findNodeIndex(const std::shared_ptr<Node>& node) const {
        auto it = std::find(nodes.begin(), nodes.end(), node);
        if (it != nodes.end()) {
            return static_cast<int>(std::distance(nodes.begin(), it));
        }
        return -1; // Node not found
    }
};

#endif // GRAPH_HPP
