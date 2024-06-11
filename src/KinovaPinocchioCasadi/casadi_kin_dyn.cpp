#include <KinovaPinocchioCasadi/casadi_kin_dyn.h>

#include <casadi/casadi.hpp>

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/math/casadi.hpp"

#include <Eigen/Dense>
#include <Eigen/Core>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#include "KinovaPinocchioCasadi/casadi_kin_dyn.h"
#include "KinovaClient/utilities.h"

#include <urdf_parser/urdf_parser.h>

// Robot Properties
std::vector<double> ratios{1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724};
std::vector<double> frictions{0.5318, 1.4776, 0.6695, 0.3013, 0.3732, 0.5923};

Eigen::VectorXd pref(6);

using namespace casadi;

namespace casadi_kin_dyn
{

    class CasadiKinDyn::Impl
    {

    public:
        Impl(std::string urdf_string);

        int nq() const;
        int nv() const;
        std::vector<double> q_min() const;
        std::vector<double> q_max() const;
        std::vector<std::string> joint_names() const;

        std::string rnea();
        std::string computeCentroidalDynamics();
        std::string ccrba();
        std::string fk(std::string link_name);
        std::string centerOfMass();
        std::string jacobian(std::string link_name, ReferenceFrame ref);
        std::string frameVelocity(std::string link_name, ReferenceFrame ref);
        std::string frameAcceleration(std::string link_name, ReferenceFrame ref);
        std::string crba();
        std::string kineticEnergy();
        std::string potentialEnergy();
        std::string aba();

        std::vector<double> computeGravity();
        std::vector<double> cartesianImpedance();
        std::vector<double> compensateFrictionInMovingDirection();
        std::vector<double> compensateFrictionInCurrentDirection();
        std::vector<double> compensateFrictionInImpedanceMode(std::vector<double> current);
        std::vector<double> NullSpaceTask();
        std::vector<double> CommandBase();
        std::vector<double> CommandBaseDirection();
        std::vector<double> CommandBaseRotation();

        void set_q(const std::vector<double> &joint_positions);
        void set_qdot(const std::vector<double> &joint_velocities);
        void set_qddot(const std::vector<double> &joint_accelerations);
        void set_tau(const std::vector<double> &joint_torques);
        void set_current(const std::vector<double> &joint_currrents);

        void set_targetx();

    private:
        typedef casadi::SX Scalar;
        typedef Eigen::Matrix<Scalar, -1, 1> VectorXs;
        typedef Eigen::Matrix<Scalar, -1, -1> MatrixXs;
        typedef Eigen::Matrix<double, -1, 1> VectorXd;

        static VectorXs cas_to_eig(const casadi::SX &cas);
        static casadi::SX eig_to_cas(const VectorXs &eig);
        static casadi::SX eigmat_to_cas(const MatrixXs &eig);

        static VectorXd vectorToEigen(const std::vector<double> &vec);

        // // custom
        // static double calculateNorm(const std::vector<double> &vec);

        pinocchio::Model _model_dbl;
        casadi::SX _q, _qdot, _qddot, _tau, _current;
        std::vector<double> _q_min, _q_max;

        std::vector<double> target_x;

        std::vector<double> x_e{0.0, 0.0, 0.0};
        std::vector<double> dx_e{0.0, 0.0, 0.0};
        std::vector<double> dx_d{0.0, 0.0, 0.0};

        Eigen::Vector3d dx;

        // Base
        std::vector<double> base_vel{0.0, 0.0, 0.0, 0.0};

        // Robot Properties
        // Eigen::VectorXd ratios{1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724};
        // Eigen::VectorXd frictions{0.5318, 1.4776, 0.6695, 0.3013, 0.3732, 0.5923};

        // // Robot Properties
        // std::vector<double> ratios{1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724};
        // std::vector<double> frictions{0.5318, 1.4776, 0.6695, 0.3013, 0.3732, 0.5923};

        //////  Define the scalar values
        //// Cartesian impedance:
        double Kd_scalar = 40.0;
        double Dd_scalar = 3.0;
        double thr_cart_error = 0.001; // m
        double error_cart_MAX = 0.1;   // m
        double thr_dynamic = 0.3;      // rad/s

        // Create identity matrices and scale them
        Eigen::MatrixXd Kd = Kd_scalar * Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd Dd = Dd_scalar * Eigen::MatrixXd::Identity(3, 3);

        //// Null space
        double K_n_scalar = 0.125;
        double D_n_scalar = 0.025;

        // Create identity matrices and scale them
        Eigen::MatrixXd K_n = K_n_scalar * Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd D_n = D_n_scalar * Eigen::MatrixXd::Identity(6, 6);

        //// Base
        double thr_pos_error = 0.05; // m
        double thr_rot_error = DEG_TO_RAD(10);
        double K_pos = 3;
        double gain_pos_MAX = 0.3;
        double K_rot = 0.2;
        double gain_rot_MAX = 0.2;
    };

    // CasadiKinDyn::Impl::Impl(urdf::ModelInterfaceSharedPtr urdf_model)
    CasadiKinDyn::Impl::Impl(std::string urdf_string)
    {

        // Load the URDF model
        std::cout << "Find urdf" << std::endl;
        pinocchio::urdf::buildModel(urdf_string, _model_dbl);
        std::cout << "Model name: " << _model_dbl.name << std::endl;

        // Add custom frame
        std::string frame_id = "END_EFFECTOR";
        std::string joint_id = "5";                       // Replace with the appropriate joint name
        std::string parent_frame_name = "GRIPPER_FRAME";  // Replace with the appropriate parent frame name
        pinocchio::SE3::Vector3 translation(0, 0, 0.156); // Set translation using Eigen's Vector3

        pinocchio::SE3 location(pinocchio::SE3::Identity()); // Initialize with identity transformation
        location.translation() = translation;                // Set translation

        pinocchio::Frame frame(frame_id, _model_dbl.getJointId(joint_id), _model_dbl.getFrameId(parent_frame_name), location, pinocchio::OP_FRAME);
        _model_dbl.addFrame(frame);

        // pinocchio::urdf::buildModel(urdf_model, _model_dbl, true); // verbose
        _q = casadi::SX::sym("q", _model_dbl.nq);
        _qdot = casadi::SX::sym("v", _model_dbl.nv);
        _qddot = casadi::SX::sym("a", _model_dbl.nv);
        _tau = casadi::SX::sym("tau", _model_dbl.nv);
        _current = casadi::SX::sym("current", _model_dbl.nv);

        _q_min.resize(_model_dbl.lowerPositionLimit.size());
        for (unsigned int i = 0; i < _model_dbl.lowerPositionLimit.size(); ++i)
            _q_min[i] = _model_dbl.lowerPositionLimit[i];

        _q_max.resize(_model_dbl.upperPositionLimit.size());
        for (unsigned int i = 0; i < _model_dbl.upperPositionLimit.size(); ++i)
            _q_max[i] = _model_dbl.upperPositionLimit[i];

        // std::cout << "_q: " << _q << std::endl;
        // std::cout << "_qdot: " << _qdot << std::endl;
        // std::cout << "_qddot: " << _qddot << std::endl;
        // std::cout << "_tau: " << _tau << std::endl;

        pref << 0, 20, 90, 0, 0, 0;

        // List of C files to compile
        std::vector<std::string> files = {
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/dq.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/dx.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/F.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/g.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/J.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/JT.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/lam.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/mu.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/N.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/Nv.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/T.c",
            "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/x.c"};

        // Output directory for .so files
        std::string output_dir = "/home/rama/Documents/cpp/KinovaCompliance/build/";

        // Compile each C file into a shared library
        for (const auto &file : files)
        {
            // Extract the file name
            size_t pos = file.find_last_of('/');
            std::string filename = file.substr(pos + 1);

            // Remove the .c extension from the file name
            filename = filename.substr(0, filename.size() - 2); // remove last two characters (.c)

            // Formulate the output path for the .so file
            std::string output_path = output_dir + filename + ".so";

            // Compile command
            std::string compile_command = "gcc -fPIC -shared -O3 " + file + " -o " + output_path;

            // Execute compilation command
            int flag = system(compile_command.c_str());

            // Check if compilation was successful
            if (flag == 0)
            {
                std::cout << "Compilation successful for " << file << std::endl;
            }
            else
            {
                std::cout << "Compilation failed for " << file << std::endl;
            }
        }
    }

    void CasadiKinDyn::Impl::set_q(const std::vector<double> &joint_positions)
    {
        std::vector<casadi::SX> q_values;
        for (const auto &val : joint_positions)
        {
            q_values.push_back(casadi::SX(val));
        }

        _q = casadi::SX::vertcat(q_values); // Convert to casadi::SX
    }

    void CasadiKinDyn::Impl::set_qdot(const std::vector<double> &joint_velocities)
    {
        std::vector<casadi::SX> qdot_values;
        for (const auto &val : joint_velocities)
        {
            qdot_values.push_back(casadi::SX(val));
        }

        _qdot = casadi::SX::vertcat(qdot_values); // Convert to casadi::SX
    }

    void CasadiKinDyn::Impl::set_qddot(const std::vector<double> &joint_accelerations)
    {
        std::vector<casadi::SX> qddot_values;
        for (const auto &val : joint_accelerations)
        {
            qddot_values.push_back(casadi::SX(val));
        }

        _qddot = casadi::SX::vertcat(qddot_values); // Convert to casadi::SX
    }

    void CasadiKinDyn::Impl::set_tau(const std::vector<double> &joint_torques)
    {
        std::vector<casadi::SX> tau_values;
        for (const auto &val : joint_torques)
        {
            tau_values.push_back(casadi::SX(val));
        }

        _tau = casadi::SX::vertcat(tau_values); // Convert to casadi::SX
    }

    void CasadiKinDyn::Impl::set_current(const std::vector<double> &joint_currents)
    {
        std::vector<casadi::SX> current_values;
        for (const auto &val : joint_currents)
        {
            current_values.push_back(casadi::SX(val));
        }

        _current = casadi::SX::vertcat(current_values); // Convert to casadi::SX
    }

    void CasadiKinDyn::Impl::set_targetx()
    {
        // auto model = _model_dbl.cast<Scalar>();
        // pinocchio::DataTpl<Scalar> data(model);

        // auto frame_idx = model.getFrameId("END_EFFECTOR");
        // pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));

        // auto x = data.oMf.at(frame_idx).translation();
        // target_x = eig_to_cas(x);

        // Var for casadi function
        DM input_q = DM(_q);
        std::vector<DM> arg_x = {input_q};

        // return pos x;
        Function x = external("x");
        std::vector<DM> res = x(arg_x);

        target_x = static_cast<std::vector<double>>(res.at(0));
    }

    std::vector<double> CasadiKinDyn::Impl::q_min() const
    {
        return _q_min;
    }

    std::vector<double> CasadiKinDyn::Impl::q_max() const
    {
        return _q_max;
    }

    std::vector<std::string> CasadiKinDyn::Impl::joint_names() const
    {
        return _model_dbl.names;
    }

    int CasadiKinDyn::Impl::nq() const
    {
        return _model_dbl.nq;
    }

    int CasadiKinDyn::Impl::nv() const
    {
        return _model_dbl.nv;
    }

    std::string CasadiKinDyn::Impl::kineticEnergy()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        Scalar DT = pinocchio::computeKineticEnergy(model, data, cas_to_eig(_q), cas_to_eig(_qdot));

        casadi::Function KINETICENERGY("kineticEnergy",
                                       {_q, _qdot}, {DT},
                                       {"q", "v"}, {"DT"});

        std::stringstream ss;
        ss << KINETICENERGY.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::potentialEnergy()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        Scalar DU = pinocchio::computePotentialEnergy(model, data, cas_to_eig(_q));

        casadi::Function POTENTIALENERGY("potentialEnergy",
                                         {_q}, {DU},
                                         {"q"}, {"DU"});

        std::stringstream ss;
        ss << POTENTIALENERGY.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::aba()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::aba(model, data, cas_to_eig(_q), cas_to_eig(_qdot), cas_to_eig(_tau));

        auto ddq = eig_to_cas(data.ddq);
        casadi::Function FD("rnea",
                            {_q, _qdot, _tau}, {ddq},
                            {"q", "v", "tau"}, {"a"});

        std::stringstream ss;
        ss << FD.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::rnea()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::rnea(model, data, cas_to_eig(_q), cas_to_eig(_qdot), cas_to_eig(_qddot));

        auto tau = eig_to_cas(data.tau);
        casadi::Function ID("rnea",
                            {_q, _qdot, _qddot}, {tau},
                            {"q", "v", "a"}, {"tau"});

        std::stringstream ss;
        ss << ID.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::computeCentroidalDynamics()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::computeCentroidalMomentumTimeVariation(model, data,
                                                          cas_to_eig(_q),
                                                          cas_to_eig(_qdot),
                                                          cas_to_eig(_qddot));

        auto h_lin = eig_to_cas(data.hg.linear());
        auto h_ang = eig_to_cas(data.hg.angular());
        auto dh_lin = eig_to_cas(data.dhg.linear());
        auto dh_ang = eig_to_cas(data.dhg.angular());
        casadi::Function CD("computeCentroidalDynamics",
                            {_q, _qdot, _qddot}, {h_lin, h_ang, dh_lin, dh_ang},
                            {"q", "v", "a"}, {"h_lin", "h_ang", "dh_lin", "dh_ang"});

        std::stringstream ss;
        ss << CD.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::ccrba()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto Ah = pinocchio::ccrba(model, data, cas_to_eig(_q), cas_to_eig(casadi::SX::zeros(nv())));
        auto Ah_cas = eigmat_to_cas(Ah);

        casadi::Function FK("ccrba",
                            {_q}, {Ah_cas},
                            {"q"}, {"A"});

        std::stringstream ss;
        ss << FK.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::frameVelocity(std::string link_name, ReferenceFrame ref)
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto frame_idx = model.getFrameId(link_name);

        // Compute expression for forward kinematics with Pinocchio
        Eigen::Matrix<Scalar, 6, -1> J;
        J.setZero(6, nv());

        pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
        // pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
        pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J);

        Eigen::Matrix<Scalar, 6, 1> eig_vel = J * cas_to_eig(_qdot);

        auto ee_vel_linear = eig_to_cas(eig_vel.head(3));
        auto ee_vel_angular = eig_to_cas(eig_vel.tail(3));

        casadi::Function FRAME_VELOCITY("frame_velocity",
                                        {_q, _qdot}, {ee_vel_linear, ee_vel_angular},
                                        {"q", "qdot"}, {"ee_vel_linear", "ee_vel_angular"});

        std::stringstream ss;
        ss << FRAME_VELOCITY.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::frameAcceleration(std::string link_name, ReferenceFrame ref)
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto frame_idx = model.getFrameId(link_name);

        // Compute expression for forward kinematics with Pinocchio
        Eigen::Matrix<Scalar, 6, -1> J, Jdot;
        J.setZero(6, nv());
        Jdot.setZero(6, nv());

        pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
        pinocchio::computeJointJacobiansTimeVariation(model, data, cas_to_eig(_q), cas_to_eig(_qdot));
        // pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));

        pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J);
        pinocchio::getFrameJacobianTimeVariation(model, data, frame_idx, pinocchio::ReferenceFrame(ref), Jdot);

        Eigen::Matrix<Scalar, 6, 1> eig_acc = J * cas_to_eig(_qddot) + Jdot * cas_to_eig(_qdot);

        auto ee_acc_linear = eig_to_cas(eig_acc.head(3));
        auto ee_acc_angular = eig_to_cas(eig_acc.tail(3));

        casadi::Function FRAME_VELOCITY("frame_acceleration",
                                        {_q, _qdot, _qddot}, {ee_acc_linear, ee_acc_angular},
                                        {"q", "qdot", "qddot"}, {"ee_acc_linear", "ee_acc_angular"});

        std::stringstream ss;
        ss << FRAME_VELOCITY.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::fk(std::string link_name)
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));

        auto frame_idx = model.getFrameId(link_name);
        auto eig_fk_pos = data.oMf.at(frame_idx).translation();
        auto eig_fk_rot = data.oMf.at(frame_idx).rotation();
        auto ee_pos = eig_to_cas(eig_fk_pos);
        auto ee_rot = eigmat_to_cas(eig_fk_rot);

        // Construct the output string with function name, arguments, and values
        std::stringstream ss;
        ss << "_q: " << _q << std::endl;
        ss << "ee_pos: [" << ee_pos << "]" << std::endl;
        ss << "ee_rot: [" << ee_rot << "]" << std::endl;

        return ss.str();
    }

    // std::string CasadiKinDyn::Impl::fk(std::string link_name)
    // {
    //     // Ensure that _q is a symbolic variable
    //     casadi::SX _q_sym = casadi::SX::sym("q", _q.size());

    //     auto model = _model_dbl.cast<Scalar>();
    //     pinocchio::DataTpl<Scalar> data(model);

    //     // Perform forward kinematics computation
    //     pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q_sym));

    //     auto frame_idx = model.getFrameId(link_name);
    //     auto eig_fk_pos = data.oMf.at(frame_idx).translation();
    //     auto eig_fk_rot = data.oMf.at(frame_idx).rotation();
    //     auto ee_pos = eig_to_cas(eig_fk_pos);
    //     auto ee_rot = eigmat_to_cas(eig_fk_rot);

    //     // Define CasADi function with _q_sym as input
    //     casadi::Function FK("forward_kinematics",
    //                         {_q_sym}, {ee_pos, ee_rot},
    //                         {"q"}, {"ee_pos", "ee_rot"});

    //     // Serialize the CasADi function to a string
    //     std::string serialized_FK = FK.serialize();
    //     std::cout << "Serialized Function: " << serialized_FK << std::endl;

    //     return serialized_FK;
    // }

    std::string CasadiKinDyn::Impl::centerOfMass()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::centerOfMass(model, data,
                                cas_to_eig(_q),
                                cas_to_eig(_qdot),
                                cas_to_eig(_qddot));

        auto com = eig_to_cas(data.com[0]);
        auto vcom = eig_to_cas(data.vcom[0]);
        auto acom = eig_to_cas(data.acom[0]);
        casadi::Function CoM("centerOfMass",
                             {_q, _qdot, _qddot}, {com, vcom, acom},
                             {"q", "v", "a"}, {"com", "vcom", "acom"});

        std::stringstream ss;
        ss << CoM.serialize();

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::jacobian(std::string link_name, ReferenceFrame ref)
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto frame_idx = model.getFrameId(link_name);

        // Compute expression for forward kinematics with Pinocchio
        Eigen::Matrix<Scalar, 6, -1> J;
        J.setZero(6, nv());

        pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
        pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
        pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J); //"LOCAL" DEFAULT PINOCCHIO COMPUTATION

        auto Jac = eigmat_to_cas(J);
        // casadi::Function JACOBIAN("jacobian", {_q}, {Jac}, {"q"}, {"J"});

        std::stringstream ss;
        // ss << JACOBIAN.serialize();
        // Construct the output string with function name, arguments, and values

        ss << "_q: " << _q << std::endl;
        ss << "Jacobian: [" << Jac << "]" << std::endl;

        return ss.str();
    }

    std::string CasadiKinDyn::Impl::crba()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto M = pinocchio::crba(model, data, cas_to_eig(_q));
        M.triangularView<Eigen::Lower>() = M.transpose();

        auto Inertia = eigmat_to_cas(M);
        casadi::Function INERTIA("crba", {_q}, {Inertia}, {"q"}, {"B"});

        std::stringstream ss;
        ss << INERTIA.serialize();

        return ss.str();
    }

    // add custom function

    // std::vector<double> CasadiKinDyn::Impl::computeGravity()
    // {
    //     auto model = _model_dbl.cast<Scalar>();
    //     pinocchio::DataTpl<Scalar> data(model);

    //     pinocchio::computeGeneralizedGravity(model, data, cas_to_eig(_q));

    //     // Construct the output string with function name, arguments, and values
    //     std::stringstream ss;
    //     ss << "_q: " << _q << std::endl;
    //     ss << "g: [" << data.g << "]" << std::endl;

    //     return ss.str();
    // }

    std::vector<double> CasadiKinDyn::Impl::computeGravity()
    {
        // auto model = _model_dbl.cast<Scalar>();
        // pinocchio::DataTpl<Scalar> data(model);

        // pinocchio::computeGeneralizedGravity(model, data, cas_to_eig(_q));

        // // Copy the elements of data.g into a std::vector<double>
        // std::vector<double> gravity(data.g.size());
        // for (size_t i = 0; i < data.g.size(); ++i)
        // {
        //     gravity[i] = static_cast<double>(data.g(i));
        // }

        // Var for casadi function
        DM input_q = DM(_q);
        std::vector<DM> arg_g = {input_q};

        // return gravity;
        Function g = external("g");
        std::vector<DM> res = g(arg_g);

        // std::cout << "res_0: " << res_0 << std::endl;

        std::vector<double> gravity = static_cast<std::vector<double>>(res.at(0));

        return gravity;
    }

    // std::vector<double> CasadiKinDyn::Impl::cartesianImpedance()
    // {
    //     auto model = _model_dbl.cast<Scalar>();
    //     pinocchio::DataTpl<Scalar> data(model);

    //     auto frame_idx = model.getFrameId("END_EFFECTOR");

    //     // Compute expression for forward kinematics with Pinocchio
    //     Eigen::Matrix<Scalar, 6, -1> J_full;
    //     J_full.setZero(6, nv());

    //     pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    //     pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
    //     pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(LOCAL_WORLD_ALIGNED), J_full); //"LOCAL" DEFAULT PINOCCHIO COMPUTATION

    //     Eigen::Matrix<Scalar, 6, 1> eig_vel = J_full * cas_to_eig(_qdot);

    //     auto ee_vel_linear = eig_to_cas(eig_vel.head(3));

    //     dx = eig_vel.head<3>().template cast<double>();

    //     std::vector<double> target_x_vec;
    //     // Iterate over each element of the SX object
    //     for (int i = 0; i < target_x.size1(); ++i)
    //     {
    //         for (int j = 0; j < target_x.size2(); ++j)
    //         {
    //             // Extract numerical value and append to the vector
    //             target_x_vec.push_back(casadi::DM(target_x(i, j)).scalar());
    //         }
    //     }

    //     auto x = data.oMf.at(frame_idx).translation();
    //     auto current_x = eig_to_cas(x);

    //     std::vector<double> current_x_vec;
    //     // Iterate over each element of the SX object
    //     for (int i = 0; i < current_x.size1(); ++i)
    //     {
    //         for (int j = 0; j < current_x.size2(); ++j)
    //         {
    //             // Extract numerical value and append to the vector
    //             current_x_vec.push_back(casadi::DM(current_x(i, j)).scalar());
    //         }
    //     }

    //     // Compute error vector
    //     std::vector<double> error;
    //     for (size_t i = 0; i < target_x_vec.size(); ++i)
    //     {
    //         error.push_back(target_x_vec[i] - current_x_vec[i]);
    //     }

    //     // Compute magnitude of error
    //     double magnitude = std::sqrt(std::pow(error[0], 2) + std::pow(error[1], 2) + std::pow(error[2], 2));

    //     // Compute x_e if magnitude is above threshold
    //     if (magnitude > thr_cart_error)
    //     {
    //         std::vector<double> vector;
    //         for (size_t i = 0; i < error.size(); ++i)
    //         {
    //             vector.push_back(error[i] / magnitude);
    //         }
    //         for (size_t i = 0; i < vector.size(); ++i)
    //         {
    //             x_e[i] = vector[i] * std::min(error_cart_MAX, magnitude);
    //         }
    //     }

    //     // Compute dx_e
    //     for (size_t i = 0; i < dx_e.size(); ++i)
    //     {
    //         dx_e[i] = dx_d[i] - dx[i];
    //     }

    //     // Compute force
    //     // std::vector<double> force;
    //     // for (int i = 0; i < Kd.rows(); ++i)
    //     // {
    //     //     double val = 0.0;
    //     //     for (int j = 0; j < Kd.cols(); ++j)
    //     //     {
    //     //         val += Kd(i, j) * x_e[j] + Dd(i, j) * dx_e[j];
    //     //     }
    //     //     force.push_back(val);
    //     // }

    //     // Check if dimensions of Kd and Dd match expected dimensions
    //     if (Kd.rows() != dx_e.size() || Kd.cols() != x_e.size() || Dd.rows() != dx_e.size() || Dd.cols() != x_e.size())
    //     {
    //         std::cerr << "Error: Dimension mismatch in Kd or Dd matrices\n";
    //         // Handle error appropriately, e.g., return or throw exception
    //     }

    //     // Compute force
    //     std::vector<double> force;
    //     for (int i = 0; i < Kd.rows(); ++i)
    //     {
    //         double val = 0.0;
    //         for (int j = 0; j < Kd.cols(); ++j)
    //         {
    //             // Check if indices are within bounds
    //             if (j < x_e.size() && j < dx_e.size())
    //             {
    //                 val += Kd(i, j) * x_e[j] + Dd(i, j) * dx_e[j];
    //             }
    //             else
    //             {
    //                 std::cerr << "Error: Index out of bounds in x_e or dx_e\n";
    //                 // Handle error appropriately, e.g., return or throw exception
    //             }
    //         }
    //         force.push_back(val);
    //     }

    //     ///////////////////////////
    //     // auto Jac = eigmat_to_cas(J);
    //     // auto T = J_EE.transpose() * force;

    //     // Compute T = J.T * f
    //     // Eigen::Vector3d f_vec(force.data(), force.data() + force.size()); /* Convert casadi::SX symbolic vector f to Eigen::Vector3d */
    //     Eigen::Vector3d f_vec(force[0], force[1], force[2]);

    //     // f_vec << FORCE_TYPEDEF
    //     // Eigen::Matrix<Scalar, 3, -1> J;
    //     // Eigen::Matrix<double, 3, Eigen::Dynamic> J; // Define J as a dynamic-size matrix
    //     // J.resize(3, nv());

    //     Eigen::Matrix<double, 3, Eigen::Dynamic> Jacob;
    //     Jacob.resize(3, nv());
    //     Jacob.setZero();

    //     // Assuming J_full is properly initialized
    //     // Jacob = J_full.topRows<3>();

    //     // Cast the elements to double
    //     Jacob = J_full.topRows<3>().template cast<double>();

    //     Eigen::Matrix<double, Eigen::Dynamic, 3> Jacob_transpose = J_full.topRows<3>().transpose().template cast<double>();

    //     // std::cout << "dx.size() : " << dx.size() << std::endl;
    //     // std::cout << "dx_d.size() : " << dx_d.size() << std::endl;
    //     // std::cout << "dx_e.size() : " << dx_e.size() << std::endl;
    //     // std::cout << "x_e.size() : " << x_e.size() << std::endl;
    //     // std::cout << "Kd.size() : " << Kd.size() << std::endl;
    //     // std::cout << "Dd.size() : " << Dd.size() << std::endl;
    //     // std::cout << "J_full.size() : " << J_full.size() << std::endl;
    //     // std::cout << J_full << std::endl;
    //     // std::cout << "Jacob.size() : " << Jacob.size() << " Jacob.rows() : " << Jacob.rows() << " Jacob.cols() : " << Jacob.cols() << std::endl;
    //     // std::cout << Jacob << std::endl;
    //     // std::cout << "Jacob_transpose.size() : " << Jacob_transpose.size() << " Jacob_transpose.rows() : " << Jacob_transpose.rows() << " Jacob_transpose.cols() : " << Jacob_transpose.cols() << std::endl;
    //     // std::cout << Jacob_transpose << std::endl;
    //     // std::cout << "f_vec.size() : " << f_vec.size() << " f_vec.rows() : " << f_vec.rows() << " f_vec.cols() : " << f_vec.cols() << std::endl;
    //     // std::cout << f_vec << std::endl;

    //     // std::cout << "target_x_vec : " << target_x_vec << std::endl;
    //     // std::cout << "current_x_vec : " << current_x_vec << std::endl;
    //     // std::cout << "error : " << error << std::endl;

    //     // Eigen::Vector3d Torq = Jacob_transpose.transpose() * f_vec;
    //     // Eigen::Vector6d Torq = f_vec.transpose() * Jacob;

    //     Eigen::VectorXd T = Jacob.transpose() * f_vec;

    //     std::vector<double> result(T.data(), T.data() + T.size());

    //     // Return the std::vector<double>
    //     return result;
    // }

    std::vector<double> CasadiKinDyn::Impl::cartesianImpedance()
    {
        x_e = {0.0, 0.0, 0.0};

        // Return pos x;
        DM input_q = DM(_q);
        std::vector<DM> arg_x = {input_q};

        Function x = external("x");
        std::vector<DM> res = x(arg_x);

        std::vector<double> current_x = static_cast<std::vector<double>>(res.at(0));

        // Compute error vector
        std::vector<double> error;
        for (size_t i = 0; i < target_x.size(); ++i)
        {
            error.push_back(target_x[i] - current_x[i]);
        }

        // std::cout << "target_x : " << target_x << std::endl;
        // std::cout << "current_x : " << current_x << std::endl;
        // std::cout << "error : " << error << std::endl;

        // Compute magnitude of error
        double magnitude = std::sqrt(std::pow(error[0], 2) + std::pow(error[1], 2) + std::pow(error[2], 2));

        // Compute x_e if magnitude is above threshold
        if (magnitude > thr_cart_error)
        {
            std::vector<double> vector;
            for (size_t i = 0; i < error.size(); ++i)
            {
                vector.push_back(error[i] / magnitude);
            }
            for (size_t i = 0; i < vector.size(); ++i)
            {
                x_e[i] = vector[i] * std::min(error_cart_MAX, magnitude);
            }
        }

        // Return vel dx;
        DM input_qdot = DM(_qdot);
        std::vector<DM> arg_dx = {input_q, input_qdot};

        Function dx = external("dx");
        res = dx(arg_dx);

        std::vector<double> current_dx = static_cast<std::vector<double>>(res.at(0));

        // Compute dx_e¥
        for (size_t i = 0; i < dx_e.size(); ++i)
        {
            dx_e[i] = dx_d[i] - current_dx[i];
        }
        // std::cout << "dx_d : " << dx_d << std::endl;
        // std::cout << "current_dx : " << current_dx << std::endl;
        // std::cout << "dx_e : " << dx_e << std::endl;

        // Compute force
        // Check if dimensions of Kd and Dd match expected dimensions
        if (Kd.rows() != dx_e.size() || Kd.cols() != x_e.size() || Dd.rows() != dx_e.size() || Dd.cols() != x_e.size())
        {
            std::cerr << "Error: Dimension mismatch in Kd or Dd matrices\n";
            // Handle error appropriately, e.g., return or throw exception
        }

        // Compute force
        std::vector<double> force;
        for (int i = 0; i < Kd.rows(); ++i)
        {
            double val = 0.0;
            for (int j = 0; j < Kd.cols(); ++j)
            {
                // Check if indices are within bounds
                if (j < x_e.size() && j < dx_e.size())
                {
                    val += Kd(i, j) * x_e[j] + Dd(i, j) * dx_e[j];
                }
                else
                {
                    std::cerr << "Error: Index out of bounds in x_e or dx_e\n";
                    // Handle error appropriately, e.g., return or throw exception
                }
            }
            force.push_back(val);
        }

        // Return T
        DM input_force = DM(force);
        std::vector<DM> arg_T = {input_q, input_force};

        Function T = external("T");
        res = T(arg_T);

        std::vector<double> torque = static_cast<std::vector<double>>(res.at(0));
        // std::cout << "torque : " << torque << std::endl;

        // Compute current
        std::vector<double> current;
        current.reserve(torque.size());

        // Multiply corresponding elements of vec1 and vec2 and store the result in result vector
        for (size_t i = 0; i < torque.size(); ++i)
        {
            // current.push_back(torque[i] * ratios[i]);
            current.push_back(torque[i]);
        }

        // std::cout << "current : " << current << std::endl;

        // std::vector<double> result = current;

        return current;
    }

    std::vector<double> CasadiKinDyn::Impl::compensateFrictionInMovingDirection()
    {
        // std::cout << "_q : " << _q << std::endl;
        // std::cout << "_qdot : " << _qdot << std::endl;
        // std::cout << "frictions : " << frictions << std::endl;

        // Check if sizes match
        if (_qdot.size1() != frictions.size())
        {
            std::cerr << "Sizes of _qdot and frictions do not match." << std::endl;
            return std::vector<double>();
        }

        // Convert _qdot to Eigen::VectorXd
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(_qdot.size1());
        for (int i = 0; i < _qdot.size1(); ++i)
        {
            dq(i) = casadi::DM(_qdot(i)).scalar();
        }

        // Compute frac_v
        Eigen::VectorXd frac_v = dq.cwiseAbs().cwiseMin(thr_dynamic) / thr_dynamic;

        // Compute sign_dq
        Eigen::VectorXd sign_dq = dq.array().sign();

        // Convert frictions to Eigen::VectorXd for element-wise multiplication
        Eigen::VectorXd frictions_vector = Eigen::VectorXd::Zero(_qdot.size1());
        for (int i = 0; i < _qdot.size1(); ++i)
        {
            frictions_vector(i) = frictions[i];
        }

        // Compute the friction compensation
        Eigen::VectorXd friction_compensation = frac_v.array() * sign_dq.array() * frictions_vector.array();

        // Convert the result back to std::vector<double>
        std::vector<double> result(friction_compensation.data(), friction_compensation.data() + friction_compensation.size());

        return result;
    }

    std::vector<double> CasadiKinDyn::Impl::compensateFrictionInCurrentDirection()
    {
        // std::cout << "_q : " << _q << std::endl;
        // std::cout << "_qdot : " << _qdot << std::endl;
        // std::cout << "frictions : " << frictions << std::endl;
        // std::cout << "_current : " << _current << std::endl;

        // Check if sizes match
        if (_qdot.size1() != frictions.size())
        {
            std::cerr << "Sizes of _qdot and frictions do not match." << std::endl;
            return std::vector<double>();
        }

        // Convert _qdot to Eigen::VectorXd
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(_qdot.size1());
        for (int i = 0; i < _qdot.size1(); ++i)
        {
            dq(i) = casadi::DM(_qdot(i)).scalar();
        }

        // Convert _current to Eigen::VectorXd
        Eigen::VectorXd current = Eigen::VectorXd::Zero(_current.size1());
        for (int i = 0; i < _current.size1(); ++i)
        {
            current(i) = casadi::DM(_current(i)).scalar();
        }

        // Compute frac_v
        Eigen::VectorXd frac_v = Eigen::VectorXd::Ones(dq.size());
        frac_v -= (dq.cwiseAbs().cwiseMin(thr_dynamic) / thr_dynamic).cwiseMin(1.0);
        // Eigen::VectorXd frac_v = dq.cwiseAbs().cwiseMin(thr_dynamic) / thr_dynamic;

        // Compute sign_dq
        Eigen::VectorXd sign_current = current.array().sign();

        // Convert frictions to Eigen::VectorXd for element-wise multiplication
        Eigen::VectorXd frictions_vector = Eigen::VectorXd::Zero(_qdot.size1());
        for (int i = 0; i < _qdot.size1(); ++i)
        {
            frictions_vector(i) = frictions[i];
        }

        // Compute the friction compensation
        Eigen::VectorXd friction_compensation = frac_v.array() * sign_current.array() * frictions_vector.array();

        // Convert the result back to std::vector<double>
        std::vector<double> result(friction_compensation.data(), friction_compensation.data() + friction_compensation.size());

        return result;
    }

    std::vector<double> CasadiKinDyn::Impl::compensateFrictionInImpedanceMode(std::vector<double> current)
    {
        set_current(current);

        // Convert current to Eigen::VectorXd
        Eigen::VectorXd current_eigen = Eigen::Map<const Eigen::VectorXd>(current.data(), current.size());

        // Get compensation in moving direction
        std::vector<double> comp_dir_mov_vec = compensateFrictionInMovingDirection();
        Eigen::VectorXd comp_dir_mov = Eigen::Map<const Eigen::VectorXd>(comp_dir_mov_vec.data(), comp_dir_mov_vec.size());

        // std::cout << "comp_dir_mov : " << comp_dir_mov << std::endl;

        // Get compensation in current direction
        std::vector<double> comp_dir_cur_vec = compensateFrictionInCurrentDirection();
        Eigen::VectorXd comp_dir_cur = Eigen::Map<const Eigen::VectorXd>(comp_dir_cur_vec.data(), comp_dir_cur_vec.size());

        // std::cout << "comp_dir_cur : " << comp_dir_cur << std::endl;

        // Combine compensations
        Eigen::VectorXd compensation = comp_dir_mov + comp_dir_cur;

        // std::cout << "compensation : " << compensation << std::endl;

        // Decrease compensation when close to target
        double sum = 0.0;
        for (double val : x_e)
        {
            sum += val * val;
        }

        double norm_x_e = std::sqrt(sum);
        compensation *= std::min(norm_x_e / thr_pos_error, 1.0);

        // Reduce compensation exponentially when current is very small compared to friction value
        Eigen::VectorXd current_squared = current_eigen.array().square();
        Eigen::ArrayXd frictions_array = Eigen::Map<const Eigen::ArrayXd>(frictions.data(), frictions.size());
        Eigen::VectorXd friction_threshold = frictions_array * 0.001;
        Eigen::VectorXd reduction_factor = current_squared.array() / friction_threshold.array();
        reduction_factor = reduction_factor.cwiseMin(1.0);
        compensation = compensation.array() * reduction_factor.array();

        // Convert result back to std::vector<double>
        std::vector<double> result(compensation.data(), compensation.data() + compensation.size());

        // std::cout << "result : " << result << std::endl;

        return result;
    }

    // std::vector<double> CasadiKinDyn::Impl::NullSpaceTask()
    // {
    //     auto model = _model_dbl.cast<Scalar>();
    //     pinocchio::DataTpl<Scalar> data(model);

    //     auto frame_idx = model.getFrameId("END_EFFECTOR");

    //     // std::cout << "Pref:" << pref << std::endl;

    //     /////////////// Compute N /////////////////
    //     // Compute the Mass matrix M using Pinocchio
    //     Eigen::Matrix<Scalar, -1, -1> M = pinocchio::crba(model, data, cas_to_eig(_q));
    //     // M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
    //     // std::cout << "M:\n"
    //     //           << M << std::endl;

    //     // Convert M into SX
    //     casadi::SX M_sx = eigmat_to_cas(M);
    //     // std::cout << "M_sx:\n"
    //     //           << M_sx << std::endl;

    //     // Compute Minv using CasADi
    //     casadi::SX Minv = casadi::SX::solve(M_sx, casadi::SX::eye(M_sx.size1())); // Inverse of M
    //     // std::cout << "Minv:\n"
    //     //           << Minv << std::endl;

    //     // Compute the Jacobian matrix J_full
    //     pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    //     pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
    //     Eigen::Matrix<Scalar, 6, -1> J_full;
    //     J_full.setZero(6, nv());
    //     pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);
    //     // std::cout << "J_full:\n"
    //     //           << J_full << std::endl;

    //     Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J;
    //     J.resize(3, nv());
    //     J.setZero();

    //     // Assuming J_full is properly initialized
    //     J = J_full.topRows<3>();
    //     // std::cout << "J:\n"
    //     //           << J << std::endl;
    //     // std::cout << "J :" << J.size() << std::endl;

    //     casadi::SX J_sx = eigmat_to_cas(J);
    //     // std::cout << "J_sx:\n"
    //     //           << J_sx << std::endl;
    //     // std::cout << "J_sx size :" << J_sx.size() << std::endl;

    //     casadi::SX J_transpose = J_sx.T();
    //     // std::cout << "J_transpose:\n"
    //     //           << J_transpose << std::endl;
    //     // std::cout << "J_transpose size :" << J_transpose.size() << std::endl;

    //     // casadi::SX Jinv = Minv * J_transpose * casadi::SX::solve((J_sx * Minv * J_transpose), casadi::SX::eye(3));
    //     // casadi::SX Jinv(6, 3);
    //     // Jinv = Minv * J_transpose;
    //     casadi::SX Minv_JT = casadi::SX::mtimes(Minv, J_transpose);
    //     // std::cout << "Minv_JT:\n"
    //     //           << Minv_JT << std::endl;
    //     // std::cout << "Minv_JT size :" << Minv_JT.size() << std::endl;

    //     casadi::SX Jsx_MinvJT = casadi::SX::mtimes(J_sx, Minv_JT);
    //     // std::cout << "Jsx_MinvJT:\n"
    //     //           << Jsx_MinvJT << std::endl;
    //     // std::cout << "Jsx_MinvJT size :" << Jsx_MinvJT.size() << std::endl;

    //     casadi::SX Jinv = casadi::SX::mtimes(Minv_JT, casadi::SX::solve(Jsx_MinvJT, casadi::SX::eye(3)));
    //     // std::cout << "Jinv:\n"
    //     //           << Jinv << std::endl;
    //     // std::cout << "Jinv size :" << Jinv.size() << std::endl;

    //     casadi::SX N = casadi::SX::eye(6) - casadi::SX::mtimes(J_transpose, Jinv.T());
    //     // casadi::SX N = casadi::SX::mtimes(J_transpose, Jinv.T());
    //     std::cout << "N:\n"
    //               << N << std::endl;
    //     std::cout << "N size :" << N.size() << std::endl;

    //     // Convert CasADi matrices to Eigen::VectorXd
    //     Eigen::VectorXd qd = pref * M_PI / 180.0; // Convert to radians
    //     Eigen::VectorXd q = cas_to_eig(_q).template cast<double>();
    //     Eigen::VectorXd qdot = cas_to_eig(_qdot).template cast<double>();

    //     // Calculate Null space stiffness and damping
    //     Eigen::VectorXd torque = ((K_n * (qd - q)) - (D_n * qdot));

    //     std::cout << "qd:\n"
    //               << qd << std::endl;

    //     std::cout << "q:\n"
    //               << q << std::endl;

    //     std::cout << "qd - q :\n"
    //               << qd - q << std::endl;

    //     std::cout << "Null space torque:\n"
    //               << torque << std::endl;
    //     std::cout << "torque size :" << torque.size() << std::endl;

    //     Eigen::VectorXd ratios_vector = vectorToEigen(ratios);
    //     Eigen::VectorXd current = torque.cwiseProduct(ratios_vector);
    //     std::cout << "Null space current:\n"
    //               << current << std::endl;
    //     std::cout << "current size :" << current.size() << std::endl;

    //     // Convert Eigen VectorXd to CasADi SX vector
    //     casadi::SX current_sx = casadi::SX::zeros(current.size(), 1); // Create SX matrix with single column
    //     for (int i = 0; i < current.size(); ++i)
    //     {
    //         current_sx(i, 0) = current(i); // Assign Eigen vector's values to SX vector
    //     }
    //     // std::cout << "current_sx:\n"
    //     //           << current_sx << std::endl;
    //     // std::cout << "current_sx size :" << current_sx.size() << std::endl;

    //     casadi::SX result_sx = casadi::SX::mtimes(N, current_sx);
    //     // std::cout << "result_sx:\n"
    //     //           << result_sx << std::endl;
    //     // std::cout << "result_sx size :" << result_sx.size() << std::endl;

    //     Eigen::VectorXd result_eig = cas_to_eig(result_sx).template cast<double>();
    //     // std::cout << "result_eig:\n"
    //     //           << result_eig << std::endl;
    //     // std::cout << "result_eig size :" << result_eig.size() << std::endl;

    //     ///////////////////////////////////////////

    //     std::vector<double> result(result_eig.data(), result_eig.data() + result_eig.size());
    //     std::cout << "result:\n"
    //               << result << std::endl;
    //     std::cout << "result size :" << result.size() << std::endl;

    //     return result;
    // }

    std::vector<double> CasadiKinDyn::Impl::NullSpaceTask()
    {
        // auto model = _model_dbl.cast<Scalar>();
        // pinocchio::DataTpl<Scalar> data(model);
        // auto frame_idx = model.getFrameId("END_EFFECTOR");

        // // Compute the Mass matrix M using Pinocchio
        // Eigen::Matrix<Scalar, -1, -1> M = pinocchio::crba(model, data, cas_to_eig(_q));
        // M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

        // // Compute Minv using CasADi
        // casadi::SX M_sx = eigmat_to_cas(M);
        // casadi::SX Minv = casadi::SX::solve(M_sx, casadi::SX::eye(M_sx.size1())); // Inverse of M

        // // Compute the Jacobian matrix J_full
        // pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
        // Eigen::Matrix<Scalar, 6, -1> J_full;
        // J_full.setZero(6, nv());
        // pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

        // Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J = J_full.topRows<3>();
        // casadi::SX J_sx = eigmat_to_cas(J);
        // casadi::SX J_transpose = J_sx.T();

        // casadi::SX Minv_JT = casadi::SX::mtimes(Minv, J_transpose);
        // casadi::SX Jsx_MinvJT = casadi::SX::mtimes(J_sx, Minv_JT);
        // casadi::SX Jinv = casadi::SX::mtimes(Minv_JT, casadi::SX::solve(Jsx_MinvJT, casadi::SX::eye(3)));

        // casadi::SX N = casadi::SX::eye(6) - casadi::SX::mtimes(J_transpose, Jinv.T());

        // // Calculate Null space stiffness and damping
        // Eigen::VectorXd qd = pref * M_PI / 180.0; // Convert to radians
        // Eigen::VectorXd q = cas_to_eig(_q).template cast<double>();
        // Eigen::VectorXd qdot = cas_to_eig(_qdot).template cast<double>();
        // Eigen::VectorXd torque = ((K_n * (qd - q)) - (D_n * qdot));
        // Eigen::VectorXd current = torque.cwiseProduct(vectorToEigen(ratios));

        // // Convert N to Eigen::MatrixXd
        // Eigen::MatrixXd N_eig = cas_to_eig(N).template cast<double>();

        // // std::cout << "N_eig:\n"
        // //           << N_eig << std::endl;

        // Eigen::VectorXd N_eig_vec = N_eig;
        // // std::cout << "N_eig_vec:\n"
        // //           << N_eig_vec << std::endl;

        // // std::cout << "current:\n"
        // //           << current << std::endl;

        // // std::cout << "Size of N_eig_vec: " << N_eig_vec.size() << std::endl;
        // // std::cout << "Size of current: " << current.size() << std::endl;
        // // std::cout << "Type of N_eig_vec: " << typeid(N_eig_vec).name() << std::endl;
        // // std::cout << "Type of current: " << typeid(current).name() << std::endl;

        // // Transpose N_vec
        // Eigen::VectorXd N_vec_transposed = N_eig_vec.transpose();

        // Eigen::VectorXd current_vec = Eigen::VectorXd::Constant(current.size(), 1, 0.0);
        // for (int i = 0; i < current.size(); ++i)
        // {
        //     current_vec(i) = current(i);
        // }

        // Eigen::VectorXd current_row = current_vec.transpose();

        // // Perform matrix-vector multiplication directly using Eigen
        // Eigen::VectorXd result;
        // if (N_vec_transposed.size() == current.size())
        // {
        //     result = N_vec_transposed.array() * current.array();
        // }
        // else
        // {
        //     std::cerr << "Error: Incompatible dimensions for element-wise multiplication." << std::endl;
        // }
        // // std::cout << "result:\n"
        // //           << result << std::endl;

        // // Convert Eigen VectorXd to std::vector<double>
        // // Convert result to std::vector<double>
        // std::vector<double> result_vec(result.data(), result.data() + result.size());
        // return result_vec;

        Eigen::VectorXd qd = pref * M_PI / 180.0; // Convert to radians
        Eigen::VectorXd q = cas_to_eig(_q).template cast<double>();
        Eigen::VectorXd qdot = cas_to_eig(_qdot).template cast<double>();
        Eigen::VectorXd torque = ((K_n * (qd - q)) - (D_n * qdot));

        // std::cout << "qd : " << qd << std::endl;
        // std::cout << "q : " << q << std::endl;
        // std::cout << "qdot : " << qdot << std::endl;
        // std::cout << "torque : " << torque << std::endl;

        // Eigen::VectorXd current_eig = torque.cwiseProduct(vectorToEigen(ratios));
        Eigen::VectorXd current_eig = torque;

        std::vector<double> current(current_eig.data(), current_eig.data() + current_eig.size());

        // Return N
        DM input_q = DM(_q);
        std::vector<DM> arg_N = {input_q};

        Function N = external("N");
        std::vector<DM> res = N(arg_N);

        std::vector<double> N_result = static_cast<std::vector<double>>(res.at(0));

        // Compute current
        std::vector<double> result;
        result.reserve(current.size());

        // Multiply corresponding elements of vec1 and vec2 and store the result in result vector
        for (size_t i = 0; i < current.size(); ++i)
        {
            // current.push_back(torque[i] * ratios[i]);
            result.push_back(N_result[i] * current[i]);
        }

        return result;
    }

    std::vector<double> CasadiKinDyn::Impl::CommandBase()
    {
        std::fill(base_vel.begin(), base_vel.end(), 0.0);

        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        auto frame_idx = model.getFrameId("END_EFFECTOR");

        pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
        pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));

        // Position:
        std::vector<double> target_x_vec;
        // Iterate over each element of the SX object
        for (int i = 0; i < target_x.size(); ++i)
        {
            // Extract numerical value and append to the vector
            target_x_vec.push_back(target_x[i]);
        }

        auto x = data.oMf.at(frame_idx).translation();
        auto current_x = eig_to_cas(x);

        std::vector<double> current_x_vec;
        // Iterate over each element of the SX object
        for (int i = 0; i < current_x.size1(); ++i)
        {
            for (int j = 0; j < current_x.size2(); ++j)
            {
                // Extract numerical value and append to the vector
                current_x_vec.push_back(casadi::DM(current_x(i, j)).scalar());
            }
        }

        std::vector<double> command(3, 0.0);

        std::vector<double> error(target_x_vec.size() - 1);
        for (size_t i = 0; i < error.size(); ++i)
        {
            error[i] = current_x_vec[i] - target_x_vec[i];
        }

        // std::cout << "Error:" << error << std::endl;

        double magnitude = std::sqrt(std::inner_product(error.begin(), error.end(), error.begin(), 0.0));
        if (magnitude > thr_pos_error)
        {
            std::vector<double> direction(error.size());
            for (size_t i = 0; i < direction.size(); ++i)
            {
                direction[i] = error[i] / magnitude;
            }
            double gain = std::min(magnitude * K_pos, gain_pos_MAX);
            command[0] = -direction[1] * gain;
            command[1] = direction[0] * gain;
        }

        // Rotation:
        double rot_error = -static_cast<double>(casadi::DM(_q(0, 0)).scalar());
        double rot_magnitude = std::abs(rot_error);
        if (rot_magnitude > thr_rot_error)
        {
            double rotation = rot_error;
            double gain = std::min(rot_magnitude * K_rot, gain_rot_MAX);
            command[2] = rot_error * gain;
        }

        return command;
    }

    // CasadiKinDyn::Impl::double CasadiKinDyn::Impl::calculateNorm(const std::vector<double> &vec)
    // {
    //     double sum = 0.0;
    //     for (double val : vec)
    //     {
    //         sum += val * val;
    //     }
    //     return std::sqrt(sum);
    // }

    // // Function to convert casadi::SX to std::vector<double>
    // std::vector<double> casadiSxToStdVector(const casadi::SX &sx)
    // {
    //     // Check if the matrix is a vector (either row or column vector)
    //     if (sx.size1() != 1 && sx.size2() != 1)
    //     {
    //         throw std::invalid_argument("Input SX is not a vector.");
    //     }

    //     // Evaluate the symbolic matrix to get a dense matrix with numerical values
    //     casadi::DM dm = casadi::DM(sx);

    //     // Determine the size of the vector
    //     size_t size = dm.size1() * dm.size2();

    //     // Create a std::vector to hold the elements
    //     std::vector<double> result(size);

    //     // Copy the elements from casadi::DM to std::vector<double>
    //     for (size_t i = 0; i < size; ++i)
    //     {
    //         result[i] = dm(i).scalar();
    //     }

    //     return result;
    // }

    // // Function to convert std::vector<double> to Eigen::VectorXd
    // Eigen::VectorXd stdVectorToEigenVector(const std::vector<double> &vec)
    // {
    //     Eigen::VectorXd eigenVec(vec.size());
    //     for (size_t i = 0; i < vec.size(); ++i)
    //     {
    //         eigenVec[i] = vec[i];
    //     }
    //     return eigenVec;
    // }

    // Convert a std::vector to an Eigen::VectorXd
    CasadiKinDyn::Impl::VectorXd CasadiKinDyn::Impl::vectorToEigen(const std::vector<double> &vec)
    {
        Eigen::VectorXd eigen_vec(vec.size());
        for (size_t i = 0; i < vec.size(); ++i)
        {
            eigen_vec(i) = vec[i];
        }
        return eigen_vec;
    }

    ///////////////////////////////////////

    CasadiKinDyn::Impl::VectorXs CasadiKinDyn::Impl::cas_to_eig(const casadi::SX &cas)
    {
        VectorXs eig(cas.size1());
        for (int i = 0; i < eig.size(); i++)
        {
            eig(i) = cas(i);
        }
        return eig;
    }

    casadi::SX CasadiKinDyn::Impl::eig_to_cas(const CasadiKinDyn::Impl::VectorXs &eig)
    {
        auto sx = casadi::SX(casadi::Sparsity::dense(eig.size()));
        for (int i = 0; i < eig.size(); i++)
        {
            sx(i) = eig(i);
        }
        return sx;
    }

    casadi::SX CasadiKinDyn::Impl::eigmat_to_cas(const CasadiKinDyn::Impl::MatrixXs &eig)
    {
        auto sx = casadi::SX(casadi::Sparsity::dense(eig.rows(), eig.cols()));
        for (int i = 0; i < eig.rows(); i++)
        {
            for (int j = 0; j < eig.cols(); j++)
            {
                sx(i, j) = eig(i, j);
            }
        }
        return sx;
    }

    CasadiKinDyn::CasadiKinDyn(std::string urdf_string)
    {
        // auto urdf = urdf::parseURDF(urdf_string);
        // _impl.reset(new Impl(urdf));

        try
        {
            auto urdf = urdf_string; // Assuming urdf_string is already a valid URDF string
            _impl.reset(new Impl(urdf));
        }
        catch (const std::exception &e)
        {
            // Print error message or throw custom exception
            std::cerr << "Error parsing URDF string: " << e.what() << std::endl;
            // You can throw an exception here or handle the error in another way
        }
    }

    int CasadiKinDyn::nq() const
    {
        return impl().nq();
    }

    int CasadiKinDyn::nv() const
    {
        return impl().nv();
    }

    std::string CasadiKinDyn::rnea()
    {
        return impl().rnea();
    }

    std::string CasadiKinDyn::computeCentroidalDynamics()
    {
        return impl().computeCentroidalDynamics();
    }

    std::string CasadiKinDyn::ccrba()
    {
        return impl().ccrba();
    }

    std::string CasadiKinDyn::crba()
    {
        return impl().crba();
    }

    std::string CasadiKinDyn::aba()
    {
        return impl().aba();
    }

    std::string CasadiKinDyn::fk(std::string link_name)
    {
        return impl().fk(link_name);
    }

    std::string CasadiKinDyn::frameVelocity(std::string link_name, ReferenceFrame ref)
    {
        return impl().frameVelocity(link_name, ref);
    }

    std::string CasadiKinDyn::frameAcceleration(std::string link_name, ReferenceFrame ref)
    {
        return impl().frameAcceleration(link_name, ref);
    }

    std::string CasadiKinDyn::centerOfMass()
    {
        return impl().centerOfMass();
    }

    std::string CasadiKinDyn::jacobian(std::string link_name, ReferenceFrame ref)
    {
        return impl().jacobian(link_name, ref);
    }

    CasadiKinDyn::~CasadiKinDyn()
    {
    }

    const CasadiKinDyn::Impl &CasadiKinDyn::impl() const
    {
        return *_impl;
    }

    CasadiKinDyn::Impl &CasadiKinDyn::impl()
    {
        return *_impl;
    }

    std::string CasadiKinDyn::kineticEnergy()
    {
        return impl().kineticEnergy();
    }

    std::string CasadiKinDyn::potentialEnergy()
    {
        return impl().potentialEnergy();
    }

    // Custom function
    std::vector<double> CasadiKinDyn::computeGravity()
    {
        return impl().computeGravity();
    }

    std::vector<double> CasadiKinDyn::cartesianImpedance()
    {
        return impl().cartesianImpedance();
    }

    std::vector<double> CasadiKinDyn::compensateFrictionInMovingDirection()
    {
        return impl().compensateFrictionInMovingDirection();
    }

    std::vector<double> CasadiKinDyn::compensateFrictionInCurrentDirection()
    {
        return impl().compensateFrictionInCurrentDirection();
    }

    std::vector<double> CasadiKinDyn::compensateFrictionInImpedanceMode(std::vector<double> current)
    {
        return impl().compensateFrictionInImpedanceMode(current);
    }

    std::vector<double> CasadiKinDyn::NullSpaceTask()
    {
        return impl().NullSpaceTask();
    }

    std::vector<double> CasadiKinDyn::CommandBase()
    {
        return impl().CommandBase();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> CasadiKinDyn::q_min() const
    {
        return impl().q_min();
    }

    std::vector<double> CasadiKinDyn::q_max() const
    {
        return impl().q_max();
    }

    std::vector<std::string> CasadiKinDyn::joint_names() const
    {
        return impl().joint_names();
    }

    void CasadiKinDyn::set_q(const std::vector<double> &joint_positions)
    {
        _impl->set_q(joint_positions);
    }

    void CasadiKinDyn::set_qdot(const std::vector<double> &joint_velocities)
    {
        _impl->set_qdot(joint_velocities);
    }

    void CasadiKinDyn::set_qddot(const std::vector<double> &joint_accelerations)
    {
        _impl->set_qddot(joint_accelerations);
    }

    void CasadiKinDyn::set_tau(const std::vector<double> &joint_torques)
    {
        _impl->set_tau(joint_torques);
    }

    void CasadiKinDyn::set_current(const std::vector<double> &joint_currents)
    {
        _impl->set_current(joint_currents);
    }

    // void CasadiKinDyn::set_targetx(const std::vector<double> &input_target_x)
    // {
    //     _impl->set_targetx(input_target_x);
    // }
    void CasadiKinDyn::set_targetx()
    {
        _impl->set_targetx();
    }

}