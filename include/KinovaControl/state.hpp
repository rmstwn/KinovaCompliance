#ifndef STATE_HPP
#define STATE_HPP

#include <iostream>
#include <vector>
#include "casadi/casadi.hpp"
#include <array>

class JointData {
public:
    int n;
    std::vector<double> q, dq, c, fault;

    JointData(int num);

    void initialize();
};

class Controller {
public:
    Controller();

    void reset();
};

class State {
private:
    casadi::Function casadi_g, casadi_x, casadi_dx, casadi_J, casadi_JT, casadi_lam, casadi_mu, casadi_N, casadi_Nv, casadi_dq, casadi_T;

public:
    JointData kinova_feedback;
    Controller controller;

    State(bool simulate);

    casadi::DM g();
    casadi::DM x();
    casadi::DM dx();
    casadi::DM J();
    casadi::DM JT();
    casadi::DM lam();
    casadi::DM mu();
    casadi::DM N();
    casadi::DM Nv();

    std::vector<double> dq_inv(std::vector<double> dx);
    std::vector<double> T(std::vector<double> force);

    void load_symbolics();
};

#endif // STATE_HPP
