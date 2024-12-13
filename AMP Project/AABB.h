#ifndef AABB_HPP
#define AABB_HPP

#include <iostream>

// Axis Aligned Bounding Box for collision checking
class AABB {
public:
    AABB(double x_min_, double x_max_, double y_min_, double y_max_, double z_min_, double z_max_)
        : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_), z_min(z_min_), z_max(z_max_) {};

    double x_min, x_max, y_min, y_max, z_min, z_max;

    static bool areTwoAABBsColliding(const AABB& box1, const AABB& box2) {
            
            // Check for separation along x axis
        if (box1.x_max < box2.x_min || box1.x_min > box2.x_max) {
            // std::cout << "Separation detected on X-axis.\n";
            return false;
        }

        // Check for separation along y axis
        if (box1.y_max < box2.y_min || box1.y_min > box2.y_max) {
            // std::cout << "Separation detected on Y-axis.\n";
            return false;
        }

        // Check for separation along z axis
        if (box1.z_max < box2.z_min || box1.z_min > box2.z_max) {
            // std::cout << "Separation detected on Z-axis.\n";
            return false;
        }

        // std::cout << "Collision detected.\n";
        return true;
    }

    void print() const {
        std::cout << "\033[34m AABB: " << "\n\033[0m";
        std::cout << "x_min: " << x_min << ", x_max: " << x_max << std::endl;
        std::cout << "y_min: " << y_min << ", y_max: " << y_max << std::endl;
        std::cout << "z_min: " << z_min << ", z_max: " << z_max << std::endl;
    }
};

#endif // AABB_HPP