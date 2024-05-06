#include "KinovaControl/state.hpp"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>
#include "KinovaControl/controller.hpp"

void JointData::initialize()
{
    q = Eigen::VectorXd::Zero(n);
    dq = Eigen::VectorXd::Zero(n);
    c = Eigen::VectorXd::Zero(n);
    fault = Eigen::VectorXd::Zero(n);
}

State::State(bool simulate)
    : kinova_feedback(JOINTS), dingo_feedback(WHEELS), kinova_command(JOINTS), dingo_command(WHEELS)
{
    // Initialize other members
    // Assign values to array members individually
    for (int i = 0; i < DIM; ++i)
    {
        target[i] = x()[i];
        absolute_target[i] = x()[i];
    }

    // Initialize pos_base and quat_base
    std::fill_n(pos_base, 3, 0.0);
    std::fill_n(quat_base, 4, 0.0);

    // Create and reset controller
    controller.reset();

    // Assign values to ratios and frictions based on simulation
    if (simulate)
    {
        std::fill_n(ratios, JOINTS, 1.0);
        std::fill_n(frictions, JOINTS, 0.2);
    }
    else
    {
        double ratiosArr[] = {1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724};
        double frictionsArr[] = {0.5318, 1.4776, 0.6695, 0.3013, 0.3732, 0.5923};
        std::copy(std::begin(ratiosArr), std::end(ratiosArr), ratios);
        std::copy(std::begin(frictionsArr), std::end(frictionsArr), frictions);
    }
}

Eigen::Vector3d State::getPosBase() const
{
    return Eigen::Vector3d(pos_base[0], pos_base[1], pos_base[2]);
}

void State::setPosBase(const Eigen::Vector3d &pos)
{
    pos_base[0] = pos[0];
    pos_base[1] = pos[1];
    pos_base[2] = pos[2];
}

Eigen::Vector4d State::getQuatBase() const
{
    return Eigen::Vector4d(quat_base[0], quat_base[1], quat_base[2], quat_base[3]);
}

void State::setQuatBase(const Eigen::Vector4d &quat)
{
    quat_base[0] = quat[0];
    quat_base[1] = quat[1];
    quat_base[2] = quat[2];
    quat_base[3] = quat[3];
}

Eigen::VectorXd State::g() {
    // return casadi_g(kinova_feedback.q);
}

Eigen::VectorXd State::x() {
    // return casadi_x(kinova_feedback.q);
}

Eigen::VectorXd State::dx() {
    // return casadi_dx(kinova_feedback.q, kinova_feedback.dq);
}

Eigen::MatrixXd State::J() {
    // return casadi_J(kinova_feedback.q).reshape(DIM, JOINTS);
}

Eigen::MatrixXd State::JT() {
    // return casadi_JT(kinova_feedback.q).reshape(JOINTS, DIM);
}

Eigen::MatrixXd State::lam() {
    // return casadi_lam(kinova_feedback.q).reshape(DIM, DIM);
}

Eigen::MatrixXd State::mu() {
    // return casadi_mu(kinova_feedback.q, kinova_feedback.dq).reshape(DIM, DIM);
}

Eigen::MatrixXd State::N() {
    // return casadi_N(kinova_feedback.q).reshape(JOINTS, JOINTS);
}

Eigen::MatrixXd State::Nv() {
    // return casadi_Nv(kinova_feedback.q).reshape(JOINTS, JOINTS);
}

Eigen::VectorXd State::dq_inv(const Eigen::VectorXd& dx) {
    // return casadi_dq(kinova_feedback.q, dx);
}

Eigen::VectorXd State::T(const Eigen::VectorXd& force) {
    // return casadi_T(kinova_feedback.q, force);
}