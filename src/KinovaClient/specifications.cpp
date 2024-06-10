#include "KinovaClient/specifications.hpp"
#include <iostream>
#include <cmath>

const float CURRENT_TORQUE_RATIOS[] = {0.85, 0.25, 0.85, 1.75, 1.75, 1.75};

std::unordered_map<int, std::vector<float>> position = {
    {0, {-159, 159}},
    {1, {-159, 159}},
    {2, {-159, 159}},
    {3, {-154, 154}},
    {4, {-154, 154}},
    {5, {-154, 154}}
};

std::unordered_map<int, std::vector<float>> velocity = {
    {0, {0, 250}},
    {1, {0, 250}},
    {2, {0, 250}},
    {3, {0, 250}},
    {4, {0, 250}},
    {5, {0, 250}}
};

std::unordered_map<int, std::vector<float>> current_motor = {
    {0, {0, 11}},
    {1, {0, 11}},
    {2, {0, 11}},
    {3, {0, 11}},
    {4, {0, 11}},
    {5, {0, 11}}
};

std::unordered_map<int, std::vector<float>> torque;

std::unordered_map<int, std::vector<float>> temperature_motor = {
    {0, {0, 60}},
    {1, {0, 60}},
    {2, {0, 60}},
    {3, {0, 60}},
    {4, {0, 60}},
    {5, {0, 60}}
};

std::unordered_map<int, std::vector<float>> temperature_core = {
    {0, {0, 80}},
    {1, {0, 80}},
    {2, {0, 80}},
    {3, {0, 80}},
    {4, {0, 80}},
    {5, {0, 80}}
};

std::unordered_map<std::string, std::unordered_map<int, std::vector<float>>> ranges = {
    {"position", position},
    {"velocity", velocity},
    {"current_motor", current_motor},
    {"torque", torque},
    {"temperature_motor", temperature_motor},
    {"temperature_core", temperature_core}
};

std::unordered_map<int, int> actuator_ids = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 4},
    {4, 5},
    {5, 7}
};

Position::Position(std::string name, std::vector<float> position) {
    this->name = name;
    this->position = position;
}

void adjustPosition(int joint, double& position) {
    float lower_bound = ranges["position"][joint][0];
    float upper_bound = ranges["position"][joint][1];
    if (position > upper_bound) {
        position -= 360;
    }
}
