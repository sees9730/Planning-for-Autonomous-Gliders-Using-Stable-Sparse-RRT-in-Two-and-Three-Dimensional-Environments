#ifndef CONTROLSURFACES_HPP
#define CONTROLSURFACES_HPP

#include <random>

class ControlSurfaces{
public:
    ControlSurfaces(double delta_e_ = 0.0, double delta_a_ = 0.0, double delta_r_ = 0.0) :
        delta_e(delta_e_), delta_a(delta_a_), delta_r(delta_r_) {};

    double delta_e; // Radians
    double delta_a; // Radians
    double delta_r; // Radians

    ControlSurfaces sampleRandomControl() const {
        ControlSurfaces control;
        control.delta_e = randomInRange(-M_PI/6, M_PI/6);
        control.delta_a = randomInRange(-M_PI/6, M_PI/6);
        control.delta_r = randomInRange(-M_PI/6, M_PI/6);
        return control;
    }

    double randomInRange(double min, double max) const {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    void print() const {
        std::cout << "\033[34m Control: \033[0m\n";
        std::cout << "Elevator Deflection: " << delta_e << "\n";
        std::cout << "Aileron Deflection: " << delta_a << "\n";
        std::cout << "Rudder Deflection: " << delta_r << "\n";
    }
};

#endif // CONTROLSURFACES