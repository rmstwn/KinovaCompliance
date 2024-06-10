#ifndef SPECIFICATIONS_HPP
#define SPECIFICATIONS_HPP

#include <unordered_map>
#include <vector>
#include <string>

// Define the Position class
class Position {
public:
    std::string name;
    std::vector<float> position;
    Position(std::string name, std::vector<float> position);
};

// Function to adjust the position
void adjustPosition(int joint, double& position);

// Extern declarations to make these variables available globally
extern const float CURRENT_TORQUE_RATIOS[];
extern std::unordered_map<int, std::vector<float>> position;
extern std::unordered_map<int, std::vector<float>> velocity;
extern std::unordered_map<int, std::vector<float>> current_motor;
extern std::unordered_map<int, std::vector<float>> torque;
extern std::unordered_map<int, std::vector<float>> temperature_motor;
extern std::unordered_map<int, std::vector<float>> temperature_core;
extern std::unordered_map<std::string, std::unordered_map<int, std::vector<float>>> ranges;
extern std::unordered_map<int, int> actuator_ids;

#endif // SPECIFICATIONS_HPP
