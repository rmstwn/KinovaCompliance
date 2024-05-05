#ifndef SPECIFICATIONS_HPP
#define SPECIFICATIONS_HPP

#include <vector>
#include <unordered_map>
#include <string>

extern const float CURRENT_TORQUE_RATIOS[];

struct Range {
    float min;
    float max;
};

class Position {
public:
    std::string name;
    std::vector<float> position;

    Position(std::string name, std::vector<float> position);
};

extern std::unordered_map<int, std::vector<float>> position;
extern std::unordered_map<int, std::vector<float>> velocity;
extern std::unordered_map<int, std::vector<float>> current_motor;
extern std::unordered_map<int, std::vector<float>> torque;
extern std::unordered_map<int, std::vector<float>> temperature_motor;
extern std::unordered_map<int, std::vector<float>> temperature_core;
extern std::unordered_map<std::string, std::unordered_map<int, std::vector<float>>> ranges;
extern std::unordered_map<int, int> actuator_ids;

#endif /* SPECIFICATIONS_HPP */
