#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <thread>
#include <cmath>
#include <Eigen/Dense>
#include "rate_counter.hpp" // Include RateCounter header

class State; // Forward declaration of State class

class Controller
{
    // Enum to represent the possible modes
    enum class Mode
    {
        POSITION,
        VELOCITY,
        CURRENT
    };

public:
    Controller();                                      // Constructor
    Controller(const Controller &) = delete;           // Delete copy constructor
    Controller &operator=(const Controller &) = delete; // Delete copy assignment operator

    // Other member functions
    void toggle(const std::string &name);
    void reset();
    void update_command();
    Eigen::VectorXd compensate_gravity();
    Eigen::VectorXd cartesian_impedance();
    Eigen::VectorXd null_space_task();
    Eigen::VectorXd compensate_friction_in_impedance_mode(Eigen::VectorXd current);
    Eigen::VectorXd compensate_friction_in_current_direction(Eigen::VectorXd current);
    Eigen::VectorXd compensate_friction_in_moving_direction();
    void command_base();
    void reset_base_command();
    void command_base_direction(const Eigen::Vector2d &direction, double gain = 1.0);
    void command_base_rotation(double rotation, double gain = 1.0);
    void start_control_loop();
    void stop_control_loop();
    void control_loop();

private:
    Eigen::VectorXd joint_commands;
    Eigen::VectorXd c_compliant;
    Eigen::VectorXd c_nullspace;
    Eigen::VectorXd c_compensate;

    double thr_cart_error;
    Eigen::Matrix3d Kd;
    Eigen::Matrix3d Dd;
    double error_cart_MAX;
    double thr_dynamic;
    Eigen::MatrixXd K_n;
    Eigen::MatrixXd D_n;
    double thr_pos_error;
    double thr_rot_error;
    double K_pos;
    double gain_pos_MAX;
    double K_rot;
    double gain_rot_MAX;

    // Eigen::Vector3d dx_d;
    // Eigen::Vector3d ddx_d;
    // Eigen::Vector3d pref_x;
    // Eigen::Vector3d x_e;
    // Eigen::Vector3d dx_e;

    bool active;
    bool comp_grav;
    bool comp_fric;
    bool imp_arm;
    bool imp_null;
    bool imp_base;

    RateCounter rate_counter; // Using RateCounter class
    State *state;             // Assuming a State class exists

    Mode mode = Mode::CURRENT; // Default mode is CURRENT
};

#endif // CONTROLLER_HPP
