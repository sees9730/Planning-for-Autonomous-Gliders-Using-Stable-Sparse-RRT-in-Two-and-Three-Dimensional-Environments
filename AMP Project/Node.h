#ifndef NODE_HPP
#define NODE_HPP

#include "State.h"
#include <vector>
#include <memory>
#include <algorithm>

struct Node {
    State state;
    ControlSurfaces control;
    double duration;
    double cost;
    bool active;
    std::shared_ptr<Node> parent; // Add this line
    std::vector<std::shared_ptr<Node>> children; // Rename neighbors to children
};
#endif // NODE_HPP