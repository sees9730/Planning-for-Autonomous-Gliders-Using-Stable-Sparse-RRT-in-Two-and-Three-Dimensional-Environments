#ifndef OBSTACLE3D_H
#define OBSTACLE3D_H

#include <vector>
#include "AABB.h"

class Obstacle3D{
public:
    // Constructor to initialize the obstacle
    Obstacle3D(double x_, double y_, double z_, double w, double h, double l)
        : x(x_), y(y_), z(z_), width(w), height(h), length(l),
          aabb(x - l / 2.0, x + l / 2.0, y - w / 2.0, y + w / 2.0, z - h / 2.0, z + h / 2.0){

            // Calculate vertices based on center position and dimensions
            double half_w = width / 2.0;
            double half_h = height / 2.0;
            double half_l = length / 2.0;

            // Define the 8 vertices of the rectangular prism
            vertices.push_back({x - half_l, y - half_w, z - half_h});
            vertices.push_back({x + half_l, y - half_w, z - half_h});
            vertices.push_back({x + half_l, y + half_w, z - half_h});
            vertices.push_back({x - half_l, y + half_w, z - half_h});
            vertices.push_back({x - half_l, y - half_w, z + half_h});
            vertices.push_back({x + half_l, y - half_w, z + half_h});
            vertices.push_back({x + half_l, y + half_w, z + half_h});
            vertices.push_back({x - half_l, y + half_w, z + half_h});

            // Calculate the AABB of the obstacle
            // AABB aabb = AABB(x - half_l, x + half_l, y - half_w, y + half_w, z - half_h, z + half_h);
        }
        
    // Position coordinates (center of the prism)
    double x, y, z;

    // Dimensions of the rectangular prism
    double width, height, length;

    // Function to display obstacle details
    void print() const {

        // Print each vertex
        std::cout << "\033[34m Vertices: \033[0m\n";
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& [vx, vy, vz] = vertices[i];
            std::cout << "  Vertex " << (i + 1) << ": (" << vx << ", " << vy << ", " << vz << ")\n";
        }
    }
    
    void exportObstaclesToCSV(const std::vector<Obstacle3D>& obstacles, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file for writing obstacles.");
        }

        file << "ObstacleID,VertexID,x,y,z\n";
        int obstacle_id = 0;
        for (const auto& obstacle : obstacles) {
            int vertex_id = 0;
            for (const auto& [x, y, z] : obstacle.vertices) {
                file << obstacle_id << "," << vertex_id << "," << x << "," << y << "," << z << "\n";
                ++vertex_id;
            }
            ++obstacle_id;
        }
        file.close();
        std::cout << "Obstacles exported to " << filename << "\n";
    }

    // Vertices of the obstacle
    std::vector<std::tuple<double, double, double>> vertices;

    // AABB of the obstacle
    AABB aabb;
};

#endif // OBSTACLE3D_H