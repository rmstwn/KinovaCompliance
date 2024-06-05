#include "KinovaMobile/kinova_mobile_utilities.hpp"
#include <cmath>

std::array<double, 4> direction_to_wheel_torques(const std::array<double, 2>& direction) {
    double m = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
    double angle = std::atan2(direction[1], direction[0]);

    double torque_A = calculate_torque(angle);
    double torque_B = calculate_torque(-angle);
    std::array<double, 4> torques = { torque_A, torque_B, torque_B, torque_A };
    for (auto& torque : torques) {
        torque *= m;
    }
    return torques;
}

std::array<double, 4> rotation_to_wheel_torques(double rotation) {
    double sign = (rotation > 0) - (rotation < 0); // signum function
    return { sign, -sign, sign, -sign };
}

double calculate_torque(double angle) {
    if (angle < -M_PI / 2) {
        return -1;
    } else if (angle < 0) {
        return 1 + angle * (4 / M_PI);
    } else if (angle < M_PI / 2) {
        return 1;
    } else {
        return 3 - angle * (4 / M_PI);
    }
}
