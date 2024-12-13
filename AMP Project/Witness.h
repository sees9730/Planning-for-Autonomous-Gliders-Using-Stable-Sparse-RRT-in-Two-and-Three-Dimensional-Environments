#include <vector>
#include <memory>
#include <limits>
#include "State.h"
#include "Node.h"

struct Witness {
    State state;                  // State representing the witness
    double cost;                  // Best cost to reach this region
    std::shared_ptr<Node> rep;    // Pointer to the representative node

    // Constructor
    Witness(const State& s) 
        : state(s), cost(std::numeric_limits<double>::infinity()), rep(nullptr) {}
};

class WitnessSet {
public:
    std::vector<Witness> witnesses;

    // Add a witness to the set
    void addWitness(const Witness& witness) {
        witnesses.push_back(witness);
    }

    // Find the nearest witness to a given state
    Witness* nearestWitness(const State& queryState) {
        if (witnesses.empty()) {
            return nullptr;
        }

        Witness* nearest = nullptr;
        double min_distance = std::numeric_limits<double>::max();

        for (auto& witness : witnesses) {
            double distance = computeDistance(queryState, witness.state);
            if (distance < min_distance) {
                min_distance = distance;
                nearest = &witness;
            }
        }

        return nearest;
    }

    // Function to determine if a new node is locally the best
    bool isNodeLocallyBest(const std::shared_ptr<Node>& x_new_node, double delta_s) {
        // Find the nearest witness
        Witness* s_new = nearestWitness(x_new_node->state);

        // Check if the distance exceeds the threshold delta_s
        // double distance = s_new ? computeDistance(x_new_node->state, s_new->state) : std::numeric_limits<double>::max();
        double distance = computeDistance(x_new_node->state, s_new->state);
        if (!s_new || distance > delta_s) {
            // Add x_new_node as a new witness
            Witness new_witness(x_new_node->state);
            addWitness(new_witness);
            s_new = &witnesses.back();
            // std::cout << "Added new witness at x = " << x_new_node->state.x << std::endl;
        }

        // Update the representative of the witness
        std::shared_ptr<Node> x_peer = s_new->rep;

        if (!x_peer || x_new_node->cost < x_peer->cost) {
            // s_new->rep = x_new_node;
            // s_new->cost = x_new_node->cost;
            return true;
        }

        return false;
    }

    void exportToCSV(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file for writing.");
        }

        // Write header
        file << "witness_x,witness_y,witness_z,witness_cost,"
             << "rep_x,rep_y,rep_z,rep_cost\n";

        // Write witness data
        for (const auto& witness : witnesses) {
            file << witness.state.x << "," 
                 << witness.state.y << "," 
                 << witness.state.z << "," 
                 << witness.cost << ",";

            if (witness.rep) {
                file << witness.rep->state.x << ","
                     << witness.rep->state.y << ","
                     << witness.rep->state.z << ","
                     << witness.rep->cost;
            } else {
                file << "NULL,NULL,NULL,NULL";
            }
            file << "\n";
        }

        file.close();
        // std::cout << "Witness set successfully exported to " << filename << "\n";
    }

    // Update a witness with a new representative node and cost
    // void update(const std::shared_ptr<Node>& node, double cost) {
    //     for (auto& witness : witnesses) {
    //         if (isNearby(node->state, witness.state)) {
    //             if (cost < witness.cost) {
    //                 witness.cost = cost;
    //                 witness.rep = node;
    //             }
    //         }
    //     }
    // }
    void print() const {
        if (witnesses.empty()) {
            std::cout << "The witness set is empty.\n";
            return;
        }

        std::cout << "Witness Set:\n";
        for (const auto& witness : witnesses) {
            std::cout << "  Witness State: ["
                    << witness.state.x << ", " << witness.state.y << ", " << witness.state.z << "]\n";
            std::cout << "    Cost: " << witness.cost << "\n";

            if (witness.rep) {
                std::cout << "    Representative State: ["
                        << witness.rep->state.x << ", " << witness.rep->state.y << ", " << witness.rep->state.z << "]\n";
            } else {
                std::cout << "    Representative: None\n";
            }
        }
    }

private:
    // Check if two states are "nearby" (can define distance threshold)
    // bool isNearby(const State& a, const State& b) const {
    //     return computeDistance(a, b) < distance_threshold;
    // }
    static double computeDistance(const State& a, const State& b) {
        return std::sqrt(
            std::pow(a.x - b.x, 2) +
            std::pow(a.y - b.y, 2) +
            std::pow(a.z - b.z, 2)
        );
    }
};