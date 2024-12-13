#ifndef PLANE_HPP
#define PLANE_HPP

#include <iostream>
#include "State.h"
#include "AABB.h"

class Plane{
public:
    // Constructor to initialize the Plane with default dimensions if not provided
    Plane(const State& state, double length_ = 10.0, double width_ = 5.0, double height_ = 2.0) :
        x(state.x), y(state.y), z(state.z),
        phi(state.phi), theta(state.theta), psi(state.psi),
        length(length_), width(width_), height(height_),
        aabb(getAABB(state)) // Initialize aabb using getAABB
    {}

    // Plane position and rotation
    double x, y, z;
    double phi, theta, psi;

    // Plane dimensions
    double length;  // x-dimension
    double width;   // y-dimension
    double height;  // z-dimension

    // Plane's AABB
    AABB aabb;

    // Compute the Plane's AABB based on the state
    AABB getAABB(const State& state) const {
        return AABB(
            state.x - length / 2.0, state.x + length / 2.0, 
            state.y - width / 2.0,  state.y + width / 2.0, 
            state.z - height / 2.0, state.z + height / 2.0
        );
    }

    // Update the Plane's AABB based on a new state
    void updateAABB(const State& state) {
        aabb = getAABB(state);
    }

    // Print Plane details
    void print() const {
        std::cout << "\033[34m Plane: \033[0m\n";
        std::cout << "Position -> x: " << x << ", y: " << y << ", z: " << z << std::endl;
        std::cout << "Rotation -> phi: " << phi << ", theta: " << theta << ", psi: " << psi << std::endl;
        std::cout << "Dimensions -> length: " << length << ", width: " << width << ", height: " << height << std::endl;
        aabb.print(); // Assuming AABB has a print method
    }

};

#endif // PLANE_HPP
