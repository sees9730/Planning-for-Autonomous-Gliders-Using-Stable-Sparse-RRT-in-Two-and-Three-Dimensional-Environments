#ifndef STATE_HPP
#define STATE_HPP

#include "Environment.h"

class State{
public:
    // Initialize the state
    State(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0,
          double phi_ = 0.0, double theta_ = 0.0, double psi_ = 0.0,
          double u_ = 0.0, double v_ = 0.0, double w_ = 0.0,
          double p_ = 0.0, double q_ = 0.0, double r_ = 0.0) :
        x(x_), y(y_), z(z_),
        phi(phi_), theta(theta_), psi(psi_),
        u(u_), v(v_), w(w_),
        p(p_), q(q_), r(r_) {}
    
    double& operator[](size_t index) {
        switch(index){
            case 0: return x;
            case 1: return y;
            case 2: return z;
            case 3: return phi;
            case 4: return theta;
            case 5: return psi;
            case 6: return u;
            case 7: return v;
            case 8: return w;
            case 9: return p;
            case 10: return q;
            case 11: return r;
            default:
                throw std::out_of_range("State index out of range");
        }
    }

    // Overload subscript operator for const access
    const double& operator[](size_t index) const {
        switch(index){
            case 0: return x;
            case 1: return y;
            case 2: return z;
            case 3: return phi;
            case 4: return theta;
            case 5: return psi;
            case 6: return u;
            case 7: return v;
            case 8: return w;
            case 9: return p;
            case 10: return q;
            case 11: return r;
            default:
                throw std::out_of_range("State index out of range");
        }
    }

    // Overload operators for vector arithmetic
    State operator+(const State& other) const {
        return State(x + other.x, y + other.y, z + other.z,
                    phi + other.phi, theta + other.theta, psi + other.psi,
                    u + other.u, v + other.v, w + other.w,
                    p + other.p, q + other.q, r + other.r);
    }

    State operator*(double scalar) const {
        return State(x * scalar, y * scalar, z * scalar,
                    phi * scalar, theta * scalar, psi * scalar,
                    u * scalar, v * scalar, w * scalar,
                    p * scalar, q * scalar, r * scalar);
    }


    // // Check if plane is in collision with the environment
    // bool inCollision(const State& state, const Environment& env) const {
        
    //     // Check the position of the state against environment boundaries
    //     if (state.x < env.x_min || state.x > env.x_max ||
    //         state.y < env.y_min || state.y > env.y_max ||
    //         state.z < env.z_min || state.z > env.z_max) {
    //         return true; // Out of bounds, thus in collision
    //     }

    //     // Check against each obstacle in the environment
    //     for (const Obstacle3D& obstacle : env.obstacles) {
    //         // Assume Obstacle3D has a method contains() to check if a point is inside it
    //         if (obstacle.contains(state.x, state.y, state.z)) {
    //             return true; // Collision detected
    //         }
    //     }

    //     return false; // No collision
        
    // }

    double x, y, z;         // Position
    double phi, theta, psi; // Orientation (roll, pitch, yaw)
    double u, v, w;         // Velocity in body frame
    double p, q, r;         // Angular rates

    double size() const { return 12; }

    // Print the state
    void print() const {
        std::cout << "\033[34m State: \033[0m\n";
        std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
        std::cout << "phi: " << phi << ", theta: " << theta << ", psi: " << psi << std::endl;
        std::cout << "u: " << u << ", v: " << v << ", w: " << w << std::endl;
        std::cout << "p: " << p << ", q: " << q << ", r: " << r << std::endl;
    }
    
    // // Function to compare states element-wise
    // bool operator==(const State& other) const {
    //     return x == other.x && y == other.y && z == other.z &&
    //            phi == other.phi && theta == other.theta && psi == other.psi &&
    //            u == other.u && v == other.v && w == other.w &&
    //            p == other.p && q == other.q && r == other.r;
    // }
};

#endif // STATE_HPP