#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "cmath"
#include "State.h"
#include "Plane.h"
#include "Obstacle3D.h"
#include "AABB.h"
#include "ControlSurfaces.h"

class Environment {
public:

    Environment() : x_min(0.0), x_max(300.0),
                    y_min(-100.0), y_max(10.0),
                    z_min(-1000.0), z_max(0.0),
                    phi_min(-M_PI), phi_max(M_PI),
                    theta_min(-M_PI / 2.0), theta_max(M_PI / 2.0),
                    psi_min(-M_PI), psi_max(M_PI),
                    u_min(0.0), u_max(80.0),
                    v_min(-10.0), v_max(10.0),
                    w_min(-50.0), w_max(50.0),
                    p_min(-3.0), p_max(3.0),
                    q_min(-3.0), q_max(3.0),
                    r_min(-3.0), r_max(3.0),
                    delta_e_min(-M_PI / (9.0 / 0.5)), delta_e_max(M_PI / (9.0 / 0.5)),
                    delta_a_min(-M_PI / (9.0 / 0.5)), delta_a_max(M_PI / (9.0 / 0.5)),
                    delta_r_min(-M_PI / (9.0 / 0.5)), delta_r_max(M_PI / (9.0 / 0.5)),
                    initial_state(State(0.0, 0.0, -100.0,
                                        0.0, 0.0, 0.0,
                                        20.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0)),
                    initial_control(ControlSurfaces(0.0, 0.0, 0.0)),
                    obstacles{},
                    // obstacles{Obstacle3D(50.0, 0.0, -100.0, 10.0, 20.0, 10.0)},
                    // obstacles{Obstacle3D(30.0, 0.0, -40.0, 10.0, 80.0, 30.0),
                            //   Obstacle3D(150.0, 0.0, -100.0, 10.0, 80.0, 30.0)},
                    // obstacles{Obstacle3D(120.0, 0.0, -110.0, 10.0, 50.0, 40.0),
                            //   Obstacle3D(197.5, 0.0, -100.0, 10.0, 100.0, 5.0),
                            //   Obstacle3D(190.0, 0.0, -20.0, 10.0, 40.0, 20.0)},
                    // obstacles{Obstacle3D(100.0, 0.0, -110.0, 10.0, 60.0, 1.0),
                    //           Obstacle3D(200.0, 0.0, -100.0, 10.0, 90.0, 1.0),
                    //           Obstacle3D(200.0, 0.0, -20.0, 10.0, 40.0, 1.0)},
                    // obstacles{Obstacle3D(30.0, 0.0, -100.0, 1.0, 10.0, 10.0)},
                    plane(initial_state, 10.0, 5.0, 2.0),
                    goal_state(200.0, -50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    // goal_state(100.0, 0.0, -110.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    goal_tolerance(10.0),
                    theta_tolerance(0.2),
                    velocity_tolerance(5.0),
                    sample_goal_state(true),
                    // sample_goal_state(false),
                    goal_sample_probability(0.7),
                    wind_center_x(70.0),    // Example center x-coordinate
                    wind_center_z(-80.0),    // Example center z-coordinate
                    wind_radius(50.0),       // Example radius
                    wind_vector_inertial({-0.0, 0.0, -0.0}),
                    desired_slope((goal_state.z - initial_state.z) / (goal_state.x - initial_state.x)),
                    desired_intercept(initial_state.z - desired_slope * initial_state.x){}; // Example wind vector [wind_x, wind_y, wind_z] in m/s

    // Define position bounds (design parameters in meters)
    double x_min, x_max, y_min, y_max, z_min, z_max;

    // Define rotation bounds (design parameters in radians)
    double theta_min, theta_max, phi_min, phi_max, psi_min, psi_max;

    // Define velocity bounds (design parameters in m/s)
    double u_min, u_max, v_min, v_max, w_min, w_max;

    // Define angular rate bounds (design parameters in rad/s)
    double p_min, p_max, q_min, q_max, r_min, r_max;

    // Define control bounds (design parameters in radians)
    double delta_e_min, delta_e_max, delta_a_min, delta_a_max, delta_r_min, delta_r_max;

    // Obstacles
    std::vector<Obstacle3D> obstacles;

    // State
    State initial_state;

    // Controls
    ControlSurfaces initial_control;

    // Plane
    Plane plane;

    // Goal state
    State goal_state;
    double goal_tolerance;
    double theta_tolerance;
    double velocity_tolerance;
    double goal_sample_probability;
    bool sample_goal_state;

    double desired_slope;
    double desired_intercept;

    // Wind disturbance parameters
    double wind_center_x, wind_center_z; // Center of the wind circle in x-z plane
    double wind_radius;                   // Radius of the wind circle
    std::vector<double> wind_vector_inertial; // Wind vector in inertial frame [wind_x, wind_y, wind_z]

    State sampleRandomState() {
        State state;
        if (sample_goal_state) {
            if (getRandomDouble(0.0, 1.0) < goal_sample_probability) {
                state = goal_state;
            } else {
                state.x = getRandomDouble(x_min, x_max);
                state.y = getRandomDouble(y_min, y_max);
                state.z = getRandomDouble(z_min, z_max);
                state.phi = getRandomDouble(phi_min, phi_max);
                state.theta = getRandomDouble(theta_min, theta_max);
                state.psi = getRandomDouble(psi_min, psi_max);
                state.u = getRandomDouble(u_min, u_max);
                state.v = getRandomDouble(v_min, v_max);
                state.w = getRandomDouble(w_min, w_max);
                state.p = getRandomDouble(p_min, p_max);
                state.q = getRandomDouble(q_min, q_max);
                state.r = getRandomDouble(r_min, r_max);
            }
        } else {
            // // Bias towards higher x values
            // if (getRandomDouble(0.0, 1.0) < goal_sample_probability) {
            //     state.x = getRandomDouble(x_max - 1, x_max);
            //     state.y = getRandomDouble(y_min, y_max);
            //     // state.z = getRandomDouble(-.1, 0)
            //     state.z = getRandomDouble(z_min, z_min + 1);
            // } else {
            //     state.x = getRandomDouble(x_min, x_max);
            //     state.y = getRandomDouble(y_min, y_max);
            //     state.z = getRandomDouble(z_min, z_max);
            // }

            state.x = getRandomDouble(x_min, x_max);
            state.y = getRandomDouble(y_min, y_max);
            state.z = getRandomDouble(z_min, z_max);
            state.phi = getRandomDouble(phi_min, phi_max);
            state.theta = getRandomDouble(theta_min, theta_max);
            state.psi = getRandomDouble(psi_min, psi_max);
            state.u = getRandomDouble(u_min, u_max);
            state.v = getRandomDouble(v_min, v_max);
            state.w = getRandomDouble(w_min, w_max);
            state.p = getRandomDouble(p_min, p_max);
            state.q = getRandomDouble(q_min, q_max);
            state.r = getRandomDouble(r_min, r_max);
        }
        return state;
    }

    ControlSurfaces sampleRandomControls() {
        ControlSurfaces controls;
        controls.delta_e = getRandomDouble(delta_e_min, delta_e_max);
        controls.delta_a = getRandomDouble(delta_a_min, delta_a_max);
        controls.delta_r = getRandomDouble(delta_r_min, delta_r_max);
        // controls.delta_e = 0.0;
        // controls.delta_a = 0.0;
        // controls.delta_r = 0.0;
        return controls;
    }

    double sampleRandomTimeStep() {
        return getRandomDouble(0.001, 0.1);
    }

    State interpolateState(const State& from, const State& to, double t) const {
        State interpolated;
        interpolated.x = from.x + t * (to.x - from.x);
        interpolated.y = from.y + t * (to.y - from.y);
        interpolated.z = from.z + t * (to.z - from.z);
        interpolated.phi = from.phi + t * (to.phi - from.phi);
        interpolated.theta = from.theta + t * (to.theta - from.theta);
        interpolated.psi = from.psi + t * (to.psi - from.psi);
        interpolated.u = from.u + t * (to.u - from.u);
        interpolated.v = from.v + t * (to.v - from.v);
        interpolated.w = from.w + t * (to.w - from.w);
        interpolated.p = from.p + t * (to.p - from.p);
        interpolated.q = from.q + t * (to.q - from.q);
        interpolated.r = from.r + t * (to.r - from.r);
        return interpolated;
    }

    bool isPlaneInCollision(const State& from, const State& to) const {
        if (from.z > 0.0 || to.z > 0.0) {
            return true; // Collision with the ground
        }
        
        auto isOutOfBounds = [&](const State& state) -> bool {
            return (state.x < x_min || state.x > x_max ||
                    state.y < y_min || state.y > y_max || 
                    state.z < z_min || state.z > z_max ||
                    state.phi < phi_min || state.phi > phi_max || 
                    state.theta < theta_min || state.theta > theta_max || 
                    state.psi < psi_min || state.psi > psi_max ||
                    state.u < u_min || state.u > u_max ||
                    state.v < v_min || state.v > v_max ||
                    state.w < w_min || state.w > w_max ||
                    state.p < p_min || state.p > p_max || 
                    state.q < q_min || state.q > q_max ||
                    state.r < r_min || state.r > r_max);
        };

        if (isOutOfBounds(from) || isOutOfBounds(to)) {
            return true; // Out-of-bound collision
        }

        // Determine the Euclidean distance between 'from' and 'to' positions
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double dz = to.z - from.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Define the number of interpolation steps based on distance
        int numSteps = static_cast<int>(std::ceil(distance));
        numSteps = std::max(numSteps, 10); // Ensure at least 10 steps
        // std::cout << "Number of interpolation steps: " << numSteps << std::endl;
        // Interpolate and check collision at each step
        for (int i = 0; i <= numSteps; ++i) {
            double t = static_cast<double>(i) / numSteps; // Normalized parameter [0,1]
            State interpolated = interpolateState(from, to, t);

            // Create a temporary Plane at the interpolated state
            Plane tempPlane(interpolated, plane.length, plane.width, plane.height);

            // Check collision with all obstacles
            // std::cout << "Number of obstacles: " << obstacles.size() << std::endl;
            for (const Obstacle3D& obstacle : obstacles) {
                // obstacle.print();
                if (AABB::areTwoAABBsColliding(obstacle.aabb, tempPlane.aabb)) {
                    // Collision detected along the trajectory
                    // std::cout << "Collision detected along the trajectory." << std::endl;
                    return true;
                }
            }
        }
        // std::cout << "No collision detected along the trajectory." << std::endl;
        // No collision detected along the trajectory
        return false;
    }

    double costFunction(const State& state, ControlSurfaces& control,
                    double duration) {
    
        double total_cost = 0.0; // Initialize total_cost

        // if (sample_goal_state) {

            double dx = state.x - goal_state.x;
            double dy = state.y - goal_state.y;
            double dz = state.z - goal_state.z;
            double distance_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);
            const double weight_goal = 1.0;
            double C_goal = weight_goal * distance_to_goal;
            total_cost += C_goal;

            // double C_smooth = 0.0;
            // const double weight_smooth_velocity = 0.06; // Weight for velocity smoothness
            // const double weight_smooth_angular = 0.1;  // Weight for angular rate smoothness

            // // Penalize high linear velocities
            // double velocity_penalty = weight_smooth_velocity * (std::abs(state.u) + std::abs(state.v) + std::abs(state.w));

            // // Penalize high angular rates
            // double angular_penalty = weight_smooth_angular * ( std::abs(state.p) + std::abs(state.r)) + 0.001 * std::abs(state.q);

            // C_smooth = velocity_penalty + angular_penalty;
            // total_cost += C_smooth;

            // // Calculate distance to the desired straight line
            // double distance_to_line = std::abs(desired_slope * state.x - state.z + desired_intercept) / std::sqrt(desired_slope * desired_slope + 1.0);
            // const double weight_straight = 1.0; // Adjust weight as needed
            // double straight_cost = weight_straight * distance_to_line;

            // total_cost += straight_cost;

            // Penalize ailero and rudder deflections
            // const double weight_aileron = 1.0; // Adjust weight as needed
            // const double weight_rudder = 1.0; // Adjust weight as needed
            // double aileron_cost = weight_aileron * std::abs(control.delta_a) + weight_rudder * std::abs(control.delta_r);




            // double x1 = initial_state.x;
            // double y1 = initial_state.y;
            // double z1 = initial_state.z;
            // double x2 = goal_state.x;
            // double y2 = goal_state.y;
            // double z2 = goal_state.z;

            // // Define the current state point
            // double px = state.x;
            // double py = state.y;
            // double pz = state.z;

            // // Calculate the direction vector of the line
            // double dxx = x2 - x1;
            // double dyy = y2 - y1;
            // double dzz = z2 - z1;

            // // Calculate the vector from the line start to the current state point
            // double vx = px - x1;
            // double vy = py - y1;
            // double vz = pz - z1;

            // // Calculate the dot product of the two vectors
            // double dot_product = vx * dxx + vy * dyy + vz * dzz;

            // // Calculate the squared magnitude of the line direction vector
            // double direction_magnitude_squared = dxx * dxx + dyy * dyy + dzz * dzz;

            // // Find the parameter `t` for the projection point on the line
            // double t = dot_product / direction_magnitude_squared;

            // // Clamp `t` to the range [0, 1] if the closest point must remain within the segment
            // t = std::max(0.0, std::min(1.0, t));

            // // Calculate the closest point on the line
            // double closest_x = x1 + t * dxx;
            // double closest_y = y1 + t * dyy;
            // double closest_z = z1 + t * dzz;

            // // Calculate the distance from the state point to the closest point on the line
            // double distance_to_line = std::sqrt(
            //     (px - closest_x) * (px - closest_x) +
            //     (py - closest_y) * (py - closest_y) +
            //     (pz - closest_z) * (pz - closest_z)
            // );

            // // Calculate the straight cost
            // const double weight_straight = 3.0; // Adjust weight as needed
            // double straight_cost = weight_straight * distance_to_line;

            // // Add to the total cost
            // total_cost += straight_cost;


            // // Obstacle Proximity Penalty
            // double obstacle_cost = 0.0;
            // const double obstacle_weight = 1.0; // High weight to strongly discourage near-obstacle paths
            // const double safety_distance = 15.3; // Adjust based on glider size and obstacle scale

            // for (const Obstacle3D& obstacle : obstacles) {
            //     // Compute the closest distance from the state to the obstacle's AABB
            //     double closest_dx = std::max(
            //         std::max(obstacle.aabb.x_min - state.x, 0.0),
            //         std::max(state.x - obstacle.aabb.x_max, 0.0)
            //     );

            //     double closest_dy = std::max(
            //         std::max(obstacle.aabb.y_min - state.y, 0.0),
            //         std::max(state.y - obstacle.aabb.y_max, 0.0)
            //     );

            //     double closest_dz = std::max(
            //         std::max(obstacle.aabb.z_min - state.z, 0.0),
            //         std::max(state.z - obstacle.aabb.z_max, 0.0)
            //     );

            //     // Compute Euclidean distance to closest point
            //     double distance_to_obstacle = std::sqrt(
            //         closest_dx * closest_dx + 
            //         closest_dy * closest_dy + 
            //         closest_dz * closest_dz
            //     );

            //     // Exponential penalty for getting close to obstacles
            //     if (distance_to_obstacle < safety_distance) {
            //         // Quadratic penalty that increases rapidly as you get closer
            //         double proximity_penalty = obstacle_weight * std::pow((safety_distance - distance_to_obstacle), 2);
            //         obstacle_cost += proximity_penalty;
            //     }
            // }

            // total_cost += obstacle_cost;

            // // Altitude and control costs (keep existing logic)
            // double altitude_cost = 0.0;
            // // double base_weight_altitude = 0.4;
            // double base_weight_altitude = 1.0;
            // double alpha = 2.0;
            // double epsilon = 0.0;
            // double weight_altitude = base_weight_altitude * (1.0 + alpha / (std::abs(state.z) + epsilon));
            // double altitude_loss = std::max(std::abs(initial_state.z) - std::abs(state.z), 0.0);
            // altitude_cost = weight_altitude * altitude_loss;

            // total_cost += altitude_cost;

            // // Control cost
            // double control_cost = 0.0 * (
            //     std::abs(control.delta_e) +
            //     std::abs(control.delta_a) +
            //     std::abs(control.delta_r)
            // );

            // total_cost += control_cost;

            // // Pitch rate penalty
            // double pitch_rate_penalty = -0.005;
            // double pitch_rate = std::abs(state.q);

            // total_cost += pitch_rate_penalty * pitch_rate;

            // // Combine all costs
            // total_cost = goal_cost + obstacle_cost + altitude_cost + control_cost + pitch_rate_penalty * pitch_rate;

            // return total_cost;

        // } else {
            
            // // Encourage horizontal range (increase in X)
            // double delta_x = state.x - initial_state.x;
            // double range_reward = delta_x;

            // // Penalize if the plane goes above ground level (z > 0)
            // double altitude_penalty = 0.0;
            // if (state.z > 0.0) {
            //     altitude_penalty = 1000.0; // Large penalty for being above ground
            // }

            // // Weights
            // const double weight_range = 1.0;    // Positive to keep cost positive
            // const double weight_control = 0.01;  // Reduced weight
            // const double weight_time = 0.01;     // Reduced weight
            // const double weight_altitude = 1.0;

            // // Control cost
            // double control_cost = std::abs(control.delta_e) +
            //                     std::abs(control.delta_a) +
            //                     std::abs(control.delta_r);

            // // Total cost
            // total_cost = weight_range * (x_max - range_reward) + weight_control * control_cost +
            //             weight_time * duration + weight_altitude * altitude_penalty;
                        
            // std::cout << "Total cost: " << total_cost << std::endl;

            // return total_cost;
        // }

        // double proximity_penalty = 0.0;
        // for (const Obstacle3D& obstacle : obstacles) {
        //     // Compute the distance from the state to the obstacle's AABB
        //     double dx = 0.0;
        //     if (state.x < obstacle.aabb.x_min) dx = obstacle.aabb.x_min - state.x;
        //     else if (state.x > obstacle.aabb.x_max) dx = state.x - obstacle.aabb.x_max;

        //     double dy = 0.0;
        //     if (state.y < obstacle.aabb.y_min) dy = obstacle.aabb.y_min - state.y;
        //     else if (state.y > obstacle.aabb.y_max) dy = state.y - obstacle.aabb.y_max;

        //     double dz = 0.0;
        //     if (state.z < obstacle.aabb.z_min) dz = obstacle.aabb.z_min - state.z;
        //     else if (state.z > obstacle.aabb.z_max) dz = state.z - obstacle.aabb.z_max;

        //     double distance_to_obstacle = std::sqrt(dx*dx + dy*dy + dz*dz);

        //     // Define a threshold distance for penalty
        //     double threshold = 15.0; // Adjust based on environment
        //     if (distance_to_obstacle < threshold) {
        //         proximity_penalty += (threshold - distance_to_obstacle) * 10.0; // Adjust multiplier
        //     }
        // }

        // Add proximity penalty to total cost
        // total_cost += proximity_penalty;

        return total_cost;
    }


    bool isGoalReached(const State& state) const {
        // Calculate Euclidean distance between current state and goal state
        double dx = state.x - goal_state.x;
        double dy = state.y - goal_state.y;
        double dz = state.z - goal_state.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // // Calculate absolute difference in theta (pitch angle)
        // double dtheta = std::abs(state.theta - goal_state.theta);

        // // Calculate absolute difference in velocity
        // double velocity = std::sqrt(state.u * state.u + state.v * state.v + state.w * state.w);
        // double goal_velocity = 11;
        // double dvelocity = std::abs(velocity - goal_velocity);

        // Check if both position and theta are within their respective tolerances
        return (distance <= goal_tolerance) ;//&& (dtheta <= theta_tolerance) && state.theta >= 0; //&& (dvelocity <= velocity_tolerance);
    }


    void exportObstaclesToCSV(const std::string& filename) const {
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

        // Append Wind Circle Data
        const int WIND_OBSTACLE_ID = 999; // Unique ID for wind circle
        const int NUM_WIND_POINTS = 100;   // Number of points to define the wind circle
        double center_x = wind_center_x;
        double center_z = wind_center_z;
        double radius = wind_radius;

        for (int i = 0; i < NUM_WIND_POINTS; ++i) {
            double angle = 2.0 * M_PI * i / NUM_WIND_POINTS;
            double x = center_x + radius * std::cos(angle);
            double y = 0.0; // Wind is in the x-z plane
            double z = center_z + radius * std::sin(angle);
            file << WIND_OBSTACLE_ID << "," << i << "," << x << "," << y << "," << z << "\n";
        }

        // Append Multiple Uniform Wind Vectors
        const int NUM_WIND_VECTORS = 40;         // Number of wind vectors
        const int WIND_VECTOR_BASE_ID = 1000;    // Starting ObstacleID for wind vectors
        const double WIND_VECTOR_SCALE = 2.0;   // Scaling factor for visibility

        // Initialize random number generators for uniform distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_radius(0.0, radius);
        std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);

        for (int i = 0; i < NUM_WIND_VECTORS; ++i) {
            int current_wind_id = WIND_VECTOR_BASE_ID + i;

            // Sample uniformly within the wind circle
            double u = dis_radius(gen) / radius; // Normalize radius
            double sampled_radius = radius * std::sqrt(u); // Ensure uniform area distribution
            double sampled_angle = dis_angle(gen);

            double wind_pos_x = center_x + sampled_radius * std::cos(sampled_angle);
            double wind_pos_z = center_z + sampled_radius * std::sin(sampled_angle);

            // Define start and end points for the wind vector
            double wind_start_x = wind_pos_x;
            double wind_start_z = wind_pos_z;
            double wind_end_x = wind_start_x + wind_vector_inertial[0] * WIND_VECTOR_SCALE;
            double wind_end_z = wind_start_z + wind_vector_inertial[2] * WIND_VECTOR_SCALE;

            // Export start point (VertexID=0)
            file << current_wind_id << "," << 0 << "," << wind_start_x << "," << 0.0 << "," << wind_start_z << "\n";
            // Export end point (VertexID=1)
            file << current_wind_id << "," << 1 << "," << wind_end_x << "," << 0.0 << "," << wind_end_z << "\n";
        }

        file.close();
        std::cout << "Obstacles and wind data exported to " << filename << "\n";
    }

private:

    double getRandomDouble(double min, double max) {
        static std::random_device rd; // Seed
        static std::mt19937 gen(rd()); // Mersenne Twister generator
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }
};

#endif // ENVIRONMENT_HPP
