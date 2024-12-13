#include "Environment.h"

bool Environment::isStateWithinBounds(const State& state, const Environment& env) const{

    // Check if state is within bounds of the environment
    if (state.x < env.x_min || state.x > env.x_max ||
        state.y < env.y_min || state.y > env.y_max || 
        state.z < env.z_min || state.z > env.z_max ||
        state.phi < env.phi_min || state.phi > env.phi_max || 
        state.theta < env.theta_min || state.theta > env.theta_max || 
        state.psi < env.psi_min || state.psi > env.psi_max ||
        state.u < env.u_min || state.u > env.u_max ||
        state.v < env.v_min || state.v > env.v_max ||
        state.w < env.w_min || state.w > env.w_max ||
        state.p < env.p_min || state.p > env.p_max || 
        state.q < env.q_min || state.q > env.q_max ||
        state.r < env.r_min || state.r > env.r_max) return false;
    return true;
}


bool Environment::isPathCollisionFree(const State& from, const State& to) const
{

    return true;;
}