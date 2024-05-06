#include "KinovaControl/controller.hpp" // Include the header file
#include "KinovaControl/state.hpp"

// Constructor
Controller::Controller() :
    active(false),
    comp_grav(true),
    comp_fric(false),
    imp_arm(false),
    imp_null(false),
    imp_base(false),
    mode(Mode::CURRENT),
    thr_cart_error(0.001),
    Kd(40 * Eigen::Matrix3d::Identity()), // Assuming Eigen is used for linear algebra
    Dd(3 * Eigen::Matrix3d::Identity()),
    error_cart_MAX(0.1),
    thr_dynamic(0.3),
    K_n(0.125 * Eigen::MatrixXd::Identity(6, 6)),
    D_n(0.025 * Eigen::MatrixXd::Identity(6, 6)),
    thr_pos_error(0.01),
    thr_rot_error(M_PI / 18.0), // 10 degrees in radians
    K_pos(4),
    gain_pos_MAX(1),
    K_rot(0.5),
    gain_rot_MAX(0.5),
    rate_counter(1000) {}

// Toggle controller states
void Controller::toggle(const std::string& name) {
    // if (name == "grav") {
    //     comp_grav = !comp_grav;
    // } else if (name == "fric") {
    //     comp_fric = !comp_fric;
    // } else if (name == "arm") {
    //     imp_arm = !imp_arm;
    // } else if (name == "null") {
    //     imp_null = !imp_null;
    // } else if (name == "base") {
    //     pref_x = state->x;
    //     imp_base = !imp_base;
    // }
    // reset();
}

// Reset method
void Controller::reset() {
    // dx_d.setZero();
    // ddx_d.setZero();
    // pref_x = state->x;
}

// Update command method
void Controller::update_command() {
    Eigen::VectorXd current = Eigen::VectorXd::Zero(JOINTS);

    if (imp_arm) {
        current += cartesian_impedance();
        if (comp_fric) {
            current += compensate_friction_in_impedance_mode(current);
        }
        if (imp_null) {
            current += null_space_task();
        }
        if (imp_base) {
            command_base();
        }
    } else {
        if (comp_fric) {
            current += compensate_friction_in_moving_direction();
        }
    }

    if (comp_grav) {
        current += compensate_gravity();
    }

    joint_commands = current;
}

// Method to compensate gravity
Eigen::VectorXd Controller::compensate_gravity() {
    // return state->g.cwiseProduct(state->ratios);
}

// Method for cartesian impedance control
Eigen::VectorXd Controller::cartesian_impedance() {
    // x_e.setZero();
    // Eigen::VectorXd error = state->target - state->x;
    // double magnitude = error.norm();
    
    // if (magnitude > thr_cart_error) {
    //     Eigen::VectorXd vector = error.normalized();
    //     x_e = vector * std::min(error_cart_MAX, magnitude);
    // }

    // dx_e = dx_d - state->dx;
    // Eigen::VectorXd force = Kd * x_e + Dd * dx_e;
    // Eigen::VectorXd torque = state->T(force);
    // return torque.cwiseProduct(state->ratios);
}

// Method for null space task
Eigen::VectorXd Controller::null_space_task() {
    // Eigen::VectorXd qd = Eigen::VectorXd::Deg2Rad(Position::pref.position);
    // Eigen::VectorXd torque = K_n * (qd - state->kinova_feedback.q) - D_n * state->kinova_feedback.dq;
    // Eigen::VectorXd current = torque.cwiseProduct(state->ratios);
    // return state->N * current;
}

// Method to compensate friction in impedance mode
Eigen::VectorXd Controller::compensate_friction_in_impedance_mode(Eigen::VectorXd current) {
    Eigen::VectorXd comp_dir_mov = compensate_friction_in_moving_direction();
    // Eigen::VectorXd comp_dir_cur = compensate_friction_in_current_direction(current);
    // Eigen::VectorXd compensation = comp_dir_mov + comp_dir_cur;
    // compensation *= std::min((x_e.norm() / thr_pos_error), 1.0);
    // compensation.cwiseProduct((current.array().square() / (state->frictions.array() * 0.001)).cwiseMin(1.0));
    // return compensation;
}

// Method to compensate friction in current direction
Eigen::VectorXd Controller::compensate_friction_in_current_direction(Eigen::VectorXd current) {
    // Eigen::VectorXd dq = state->kinova_feedback.dq;
    // Eigen::VectorXd frac_v = Eigen::VectorXd::Ones(JOINTS) - dq.array().abs().cwiseMin(thr_dynamic).array();
    // return frac_v.cwiseProduct(current.cwiseSign().cwiseProduct(state->frictions));
}

// Method to compensate friction in moving direction
Eigen::VectorXd Controller::compensate_friction_in_moving_direction() {
    // Eigen::VectorXd dq = state->kinova_feedback.dq;
    // Eigen::VectorXd frac_v = dq.array().abs().cwiseMin(thr_dynamic).array();
    // return frac_v.cwiseProduct(dq.cwiseSign().cwiseProduct(state->frictions));
}

// Method to command the base
void Controller::command_base() {
    reset_base_command();

    // // Position control
    // Eigen::VectorXd error = (state->x - pref_x).head<2>(); // Assuming the first two elements represent the x, y position
    // double magnitude = error.norm();
    
    // if (magnitude > thr_pos_error) {
    //     Eigen::Vector2d direction = error.normalized();
    //     double gain = std::min(magnitude * K_pos, gain_pos_MAX);
    //     command_base_direction(-direction, gain); // Assuming negative y is forward motion
    // }

    // // Rotation control
    // double rotation = -state->kinova_feedback.q[0]; // Assuming first element represents the rotation
    // double magnitude = std::abs(rotation);
    
    // if (magnitude > thr_rot_error) {
    //     double gain = std::min(magnitude * K_rot, gain_rot_MAX);
    //     command_base_rotation(rotation, gain);
    // }
}

// Method to reset base command
void Controller::reset_base_command() {
    // state->dingo_command.c.setZero();
}

// Method to command base direction
void Controller::command_base_direction(const Eigen::Vector2d& direction, double gain) {
    // state->dingo_command.c += direction_to_wheel_torques(direction) * gain; // Implement direction_to_wheel_torques function
}

// Method to command base rotation
void Controller::command_base_rotation(double rotation, double gain) {
    // state->dingo_command.c += rotation_to_wheel_torques(rotation) * gain; // Implement rotation_to_wheel_torques function
}

// Method to start the control loop
void Controller::start_control_loop() {
    active = true;
    std::thread loop_thread(&Controller::control_loop, this);
    loop_thread.detach();
}

// Method to stop the control loop
void Controller::stop_control_loop() {
    active = false;
    // state->dingo_command.c.setZero();
}

// Method for the control loop
void Controller::control_loop() {
    while (active) {
        update_command();
        rate_counter.count(); // Count the loop iteration
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(rate_counter.getSleepTime() * 1000000))); // Sleep to match desired rate
    }
}