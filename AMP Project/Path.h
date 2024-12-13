#ifndef PATH_HPP
#define PATH_HPP

#include "Node.h"
#include <vector>
#include <iostream>

class Path{
public:

    void print() const{
        std::cout << "\032[34m Path: " << "\n\033[0m";
        for (const auto& node : nodes) {
            node->print();
        }
        // std::cout << "  Nodes: " << nodes.size() << std::endl;
    }

    std::vector<std::shared_ptr<Node>> nodes;
}



#endif // PATH_HPP