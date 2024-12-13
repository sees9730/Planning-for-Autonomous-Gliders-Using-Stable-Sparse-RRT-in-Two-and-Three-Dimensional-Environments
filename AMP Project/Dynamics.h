    #ifndef DYNAMICS_HPP
    #define DYNAMICS_HPP


    class Dynamics{
    public:

        // Aircraft parameters
        double mass;
        double I_x;
        double I_y;
        double I_z;
        double g;
        double rho;
        double S;
        double c;
        double b;

        // Constructor with default values
        Dynamics(double mass = 5.6, double I_x = 0.15, double I_y = 0.3, double I_z = 0.2,
                double rho = 1.225, double S = 0.3108, double c = 0.208, double b = 0.5, double g = 9.81)
            : mass(mass),
            I_x(I_x),
            I_y(I_y),
            I_z(I_z),
            g(g),
            rho(rho),
            S(S),
            c(c),
            b(b) {}



        State computeDerivatives(const Environment& env, const State& state, double delta_e, double delta_a, double delta_r) const {
            // Unpack state variables
            double x = state.x, y = state.y, z = state.z;
            double u = state.u, v = state.v, w = state.w;
            double phi = state.phi, theta = state.theta, psi = state.psi;
            double p = state.p, q = state.q, r = state.r;

            // Check if plane is within the wind disturbance circle in x-z plane
            double dx_wind = x - env.wind_center_x;
            double dz_wind = z - env.wind_center_z;
            bool within_wind = (dx_wind * dx_wind + dz_wind * dz_wind) <= (env.wind_radius * env.wind_radius);

            // Define relative velocities
            double u_rel = u;
            double v_rel = v;
            double w_rel = w;

            if (within_wind) {
                // std::cout << "Within wind disturbance circle" << std::endl;
                // Rotate wind vector from inertial to body frame
                std::vector<double> wind_body = rotateToBodyFrame(state, env.wind_vector_inertial);
                
                // Adjust relative velocities by subtracting wind
                u_rel = u - wind_body[0];
                v_rel = v - wind_body[1];
                w_rel = w - wind_body[2];
            }

            // Compute relative airspeed and angles
        
                double V_rel = sqrt(u_rel * u_rel + v_rel * v_rel + w_rel * w_rel);
                double alpha = atan2(w_rel, u_rel);
                double beta = asin(v_rel / V_rel);

            // Aerodynamic coefficients based on relative airspeed
            double CL = computeCL(alpha);
            double CD = computeCD(alpha);
            double CY = computeCY(beta);
            double Cm = computeCm(alpha, delta_e);
            double Cl = computeCl(beta, p, r, delta_a, delta_r, V_rel);
            double Cn = computeCn(beta, p, r, delta_a, delta_r, V_rel);

            // Dynamic pressure based on relative airspeed
            double q_bar = 0.5 * rho * V_rel * V_rel;

            // Aerodynamic forces in body frame based on relative airspeed
            double L = q_bar * S * CL; // Lift
            double D = q_bar * S * CD; // Drag
            double Y = q_bar * S * CY; // Side force

            // Aerodynamic forces in body frame
            double F_ax = -D * cos(alpha) + L * sin(alpha);
            double F_ay = Y; // Assuming side force affects only lateral dynamics
            double F_az = -D * sin(alpha) - L * cos(alpha);

            // Gravitational forces in body frame
            double F_gx = -mass * g * sin(theta);
            double F_gy = mass * g * sin(phi) * cos(theta);
            double F_gz = mass * g * cos(phi) * cos(theta);

            // Total forces in body frame
            double F_x = F_ax + F_gx;
            double F_y = F_ay + F_gy;
            double F_z = F_az + F_gz;

            // Moments
            double L_moment = q_bar * S * b * Cl;
            double M_moment = q_bar * S * c * Cm;
            double N_moment = q_bar * S * b * Cn;

            // Dynamic translational equations of motion (p. 104 in book)
            double du = r * v - q * w + F_x / mass;
            double dv = p * w - r * u + F_y / mass;
            double dw = q * u - p * v + F_z / mass;

            // Dynamic rotational equations of motion (p. 104 in book)
            double dp = ((I_y - I_z) * q * r) / I_x + L_moment / I_x;
            double dq = ((I_z - I_x) * p * r) / I_y + M_moment / I_y;
            double dr = ((I_x - I_y) * p * q) / I_z + N_moment / I_z;

            // Kinematic rotational equations 
            double dphi = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
            double dtheta = q * cos(phi) - r * sin(phi);
            double dpsi = (q * sin(phi) + r * cos(phi)) / cos(theta);

            // Kinematic translational equations
            double dx_dot = u * cos(theta) * cos(psi) + v * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
            double dy_dot = u * cos(theta) * sin(psi) + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) + w * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
            double dz_dot = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta);

            // Return derivatives as a State object
            return State(dx_dot, dy_dot, dz_dot, dphi, dtheta, dpsi, du, dv, dw, dp, dq, dr);
        }

        // // Compute the derivatives of the state
        // State computeDerivatives(const Environment& env, const State& state, double delta_e, double delta_a, double delta_r) const{

        //     // Unpack state variables
        //     double x = state.x, y = state.y, z = state.z;
        //     double u = state.u, v = state.v, w = state.w;
        //     double phi = state.phi, theta = state.theta, psi = state.psi;
        //     double p = state.p, q = state.q, r = state.r;

        //     // Compute airspeed and angle of attack
        //     double V = sqrt(u * u + v * v + w * w);
        //     double alpha = atan2(w, u);
        //     double beta = asin(v / V);

        //     // Aerodynamic coefficients
        //     double CL = computeCL(alpha);
        //     double CD = computeCD(alpha);
        //     double CY = computeCY(beta);
        //     double Cm = computeCm(alpha, delta_e);
        //     double Cl = computeCl(beta, p, r, delta_a, delta_r, V);
        //     double Cn = computeCn(beta, p, r, delta_a, delta_r, V);

        //     // Dynamic pressure
        //     double q_bar = 0.5 * rho * V * V;

        //     // Aerodynamic forces
        //     double L = q_bar * S * CL; // Lift
        //     double D = q_bar * S * CD; // Drag
        //     double Y = q_bar * S * CY; // Side force

        //     // Aerodynamic forces in body frame
        //     // double F_ax = -D * cos(alpha) * cos(beta) + L * sin(alpha) - Y * sin(beta);
        //     // double F_ay = -D * cos(alpha) * sin(beta) + L * sin(alpha) + Y * cos(beta);
        //     // double F_az = -D * sin(alpha) - L * cos(alpha);
        //     double F_ax = -D * cos(alpha) + L * sin(alpha);
        //     double F_ay = 0; // Neglected in 2D
        //     double F_az = -D * sin(alpha) - L * cos(alpha);

        //     // Gravitational forces in body frame
        //     double F_gx = -mass * g * sin(theta);
        //     double F_gy = mass * g * sin(phi) * cos(theta);
        //     double F_gz = mass * g * cos(phi) * cos(theta);

        //     // Total forces in body frame
        //     double F_x = F_ax + F_gx;
        //     double F_y = F_ay + F_gy;
        //     double F_z = F_az + F_gz;

        //     // Moments
        //     double L_moment = q_bar * S * b * Cl;
        //     double M_moment = q_bar * S * c * Cm;
        //     double N_moment = q_bar * S * b * Cn;

        //     // Dynamic translational equations of motion (p. 104 in book)
        //     double du = r * v - q * w + F_x / mass;
        //     double dv = p * w - r * u + F_y / mass;
        //     double dw = q * u - p * v + F_z / mass;

        //     // std::cout << "dv: " << dv << std::endl;
        //     // std::cout << "p: " << p << std::endl;
        //     // std::cout << "w: " << w << std::endl;
        //     // std::cout << "Beta: " << beta << std::endl;
        //     // std::cout << "CL: " << CL << std::endl;
        //     // std::cout << "CD: " << CD << std::endl;
        //     // std::cout << "CY: " << CY << std::endl;
        //     // std::cout << "F_ax: " << F_ax << std::endl;
        //     // std::cout << "F_ay: " << F_ay << std::endl;
        //     // std::cout << "F_az: " << F_az << std::endl;
        //     // std::cout << "F_x: " << F_x << std::endl;
        //     // std::cout << "F_y: " << F_y << std::endl;
        //     // std::cout << "F_z: " << F_z << std::endl;
        //     // std::cout << "L_moment: " << L_moment << std::endl;
        //     // std::cout << "M_moment: " << M_moment << std::endl;
        //     // std::cout << "N_moment: " << N_moment << std::endl << std::endl;

        //     // Dynamic rotational equations of motion (p. 104 in book)
        //     double dp = ((I_y - I_z) * q * r) / I_x + L_moment / I_x;
        //     double dq = ((I_z - I_x) * p * r) / I_y + M_moment / I_y;
        //     double dr = ((I_x - I_y) * p * q) / I_z + N_moment / I_z;

        //     // Kinematic rotational equations 
        //     double dphi = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
        //     double dtheta = q * cos(phi) - r * sin(phi);
        //     double dpsi = (q * sin(phi) + r * cos(phi)) / cos(theta);

        //     // Kinematic translational equations
        //     double dx = u * cos(theta) * cos(psi) + v * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
        //     double dy = u * cos(theta) * sin(psi) + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) + w * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
        //     double dz = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta);

        //     // Return derivatives as a State object
        //     return State(dx, dy, dz, dphi, dtheta, dpsi, du, dv, dw, dp, dq, dr);
        // }

        double computeCL(double alpha) const {
            double CL_0 = 0.2;
            double CL_alpha = 5.7;  // Lift curve slope
            return CL_0 + CL_alpha * alpha;
        }

        double computeCD(double alpha) const {
            double CD_0 = 0.02;     // Zero-lift drag coefficient
            double k = 0.04;        // Induced drag factor
            double CL = computeCL(alpha);
            return CD_0 + k * CL * CL;
        }

        double computeCm(double alpha, double delta_e) const {
            double Cm_0 = 0;    // Base pitch moment coefficient
            double Cm_alpha = -0.5; // Pitch moment curve slope
            double Cm_delta_e = -1.0; // Control effectiveness
            return Cm_0 + Cm_alpha * alpha + Cm_delta_e * delta_e;
        }

        double computeCY(double beta) const {
            double CY_beta = -0.98; // Side force derivative with respect to beta
            return CY_beta * beta;
            // return 0;
        }

        double computeCl(double beta, double p, double r, double delta_a, double delta_r, double V) const {
            double Cl_beta = -0.12;
            double Cl_p = -0.26; // Roll moment derivative with respect to p
            double Cl_r = 0.14;  // Roll moment derivative with respect to r
            double Cl_delta_a = 0.08; // Aileron effectiveness
            double Cl_delta_r = 0.105; // Rudder effectiveness

            // Non-dimensionalize p and r
            double p_hat = (p * b) / (2.0 * V);
            double r_hat = (r * b) / (2.0 * V);

            return Cl_beta * beta + Cl_p * p_hat + Cl_r * r_hat + Cl_delta_a * delta_a + Cl_delta_r * delta_r;
            // return 0;
        }

        double computeCn(double beta, double p, double r, double delta_a, double delta_r, double V) const {
            double Cn_beta = 0.25;
            double Cn_p = -0.022; // Yaw moment derivative with respect to p
            double Cn_r = -0.35;  // Yaw moment derivative with respect to r
            double Cn_delta_a = 0.06; // Aileron effectiveness
            double Cn_delta_r = -0.032; // Rudder effectiveness

            // Non-dimensionalize p and r
            double p_hat = (p * b) / (2.0 * V);
            double r_hat = (r * b) / (2.0 * V);

            return Cn_beta * beta + Cn_p * p_hat + Cn_r * r_hat + Cn_delta_a * delta_a + Cn_delta_r * delta_r;
            // return 0;
        }



        State getNextState(Environment& env, const State& state, ControlSurfaces& control_surfaces, double& dt) const {
            // Sample controls and time step
            control_surfaces = env.sampleRandomControls();
            // dt = env.sampleRandomTimeStep();
            dt = 0.1;
            
            double delta_e = control_surfaces.delta_e;
            double delta_a = control_surfaces.delta_a;
            double delta_r = control_surfaces.delta_r;

            // Compute state derivatives using updated computeDerivatives
            State k1 = computeDerivatives(env, state, delta_e, delta_a, delta_r);

            // Compute k2
            State state_k1 = state + k1 * (dt * 0.5);
            State k2 = computeDerivatives(env, state_k1, delta_e, delta_a, delta_r);

            // Compute k3
            State state_k2 = state + k2 * (dt * 0.5);
            State k3 = computeDerivatives(env, state_k2, delta_e, delta_a, delta_r);

            // Compute k4
            State state_k3 = state + k3 * dt;
            State k4 = computeDerivatives(env, state_k3, delta_e, delta_a, delta_r);

            // Update state using weighted average of slopes
            State next_state = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

            return next_state;
        }

        // // Runge-Kutta 4th Order Integrator
        // State getNextState(Environment& env, const State& state, ControlSurfaces& control_surfaces, double& dt) const {

        //     // Randomly sample controls
        //     control_surfaces = env.sampleRandomControls();
        //     // control_surfaces = ControlSurfaces{0, 0, 0};
        //     // dt = env.sampleRandomTimeStep();
        //     dt = 0.1;

        //     double delta_e = control_surfaces.delta_e;
        //     double delta_a = control_surfaces.delta_a;
        //     double delta_r = control_surfaces.delta_r;

        //     // Compute k1
        //     State k1 = computeDerivatives(state, delta_e, delta_a, delta_r);

        //     // Compute k2
        //     State state_k1 = state + k1 * (dt * 0.5);
        //     State k2 = computeDerivatives(state_k1, delta_e, delta_a, delta_r);

        //     // Compute k3
        //     State state_k2 = state + k2 * (dt * 0.5);
        //     State k3 = computeDerivatives(state_k2, delta_e, delta_a, delta_r);

        //     // Compute k4
        //     State state_k3 = state + k3 * dt;
        //     State k4 = computeDerivatives(state_k3, delta_e, delta_a, delta_r);

        //     // Update state using weighted average of slopes
        //     State next_state = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

        //     return next_state;
        // }

        std::vector<double> rotateToBodyFrame(const State& state, const std::vector<double>& vec_inertial) const {
            double phi = state.phi;
            double theta = state.theta;
            double psi = state.psi;
            
            double cos_phi = cos(phi);
            double sin_phi = sin(phi);
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);
            double cos_psi = cos(psi);
            double sin_psi = sin(psi);
            
            // Rotation matrix components
            double r11 = cos_theta * cos_psi;
            double r12 = cos_theta * sin_psi;
            double r13 = -sin_theta;
            
            double r21 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
            double r22 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
            double r23 = sin_phi * cos_theta;
            
            double r31 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
            double r32 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
            double r33 = cos_phi * cos_theta;
            
            // Apply rotation
            double body_x = r11 * vec_inertial[0] + r12 * vec_inertial[1] + r13 * vec_inertial[2];
            double body_y = r21 * vec_inertial[0] + r22 * vec_inertial[1] + r23 * vec_inertial[2];
            double body_z = r31 * vec_inertial[0] + r32 * vec_inertial[1] + r33 * vec_inertial[2];
            
            return {body_x, body_y, body_z};
        }

    };






    #endif // DYNAMICS_HPP