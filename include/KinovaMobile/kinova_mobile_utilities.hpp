#ifndef KINOVA_MOBILE_UTILITIES_HPP
#define KINOVA_MOBILE_UTILITIES_HPP

#include <array>

std::array<double, 4> direction_to_wheel_torques(const std::array<double, 2>& direction);
std::array<double, 4> rotation_to_wheel_torques(double rotation);
double calculate_torque(double angle);

#endif // KINOVA_MOBILE_UTILITIES_HPP
