#ifndef STATE_HPP
#define STATE_HPP

#include <iostream>
#include <vector>
#include "casadi/casadi.hpp"
#include <array>
#include "controller.hpp" // Assuming Controller class is defined in controller.hpp

constexpr int JOINTS = 6;
constexpr int WHEELS = 4;
constexpr int DIM = 3;

class JointData
{
public:
    int n;
    Eigen::VectorXd q, dq, c, fault;

    JointData(int num) : n(num)
    {
        initialize();
    }

    void initialize();
};

class State
{
private:
    casadi::Function casadi_g, casadi_x, casadi_dx, casadi_J, casadi_JT, casadi_lam, casadi_mu, casadi_N, casadi_Nv, casadi_dq, casadi_T;

public:
    JointData kinova_feedback;
    JointData dingo_feedback;
    JointData kinova_command;
    JointData dingo_command;

    double target[DIM];
    double absolute_target[DIM];
    double pos_base[3];
    double quat_base[4];

    Controller controller;

    double ratios[JOINTS];
    double frictions[JOINTS];

    State(bool simulate);

    Eigen::VectorXd g();
    Eigen::VectorXd x();
    Eigen::VectorXd dx();
    Eigen::MatrixXd J();
    Eigen::MatrixXd JT();
    Eigen::MatrixXd lam();
    Eigen::MatrixXd mu();
    Eigen::MatrixXd N();
    Eigen::MatrixXd Nv();

    Eigen::VectorXd dq_inv(const Eigen::VectorXd &dx);
    Eigen::VectorXd T(const Eigen::VectorXd &force);

    void load_symbolics();

    Eigen::Vector3d getPosBase() const;
    void setPosBase(const Eigen::Vector3d &pos);

    Eigen::Vector4d getQuatBase() const;
    void setQuatBase(const Eigen::Vector4d &quat);
};

#endif // STATE_HPP
