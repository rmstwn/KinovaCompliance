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

#include <iostream>

#include "KinovaPinocchioCasadi/casadi_kin_dyn.h"

#include <urdf_parser/urdf_parser.h>

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

        std::string computeGravity();

        void set_q(const std::vector<double> &joint_positions);
        void set_qdot(const std::vector<double> &joint_velocities);
        void set_qddot(const std::vector<double> &joint_accelerations);
        void set_tau(const std::vector<double> &joint_currrents);

    private:
        typedef casadi::SX Scalar;
        typedef Eigen::Matrix<Scalar, -1, 1> VectorXs;
        typedef Eigen::Matrix<Scalar, -1, -1> MatrixXs;

        static VectorXs cas_to_eig(const casadi::SX &cas);
        static casadi::SX eig_to_cas(const VectorXs &eig);
        static casadi::SX eigmat_to_cas(const MatrixXs &eig);

        pinocchio::Model _model_dbl;
        casadi::SX _q, _qdot, _qddot, _tau;
        std::vector<double> _q_min, _q_max;
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

    void CasadiKinDyn::Impl::set_tau(const std::vector<double> &joint_currents)
    {

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

    // add custom function;
    std::string CasadiKinDyn::Impl::computeGravity()
    {
        auto model = _model_dbl.cast<Scalar>();
        pinocchio::DataTpl<Scalar> data(model);

        pinocchio::computeGeneralizedGravity(model, data, cas_to_eig(_q));

        // Construct the output string with function name, arguments, and values
        std::stringstream ss;
        ss << "_q: " << _q << std::endl;
        ss << "g: [" << data.g << "]" << std::endl;

        return ss.str();
    }

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
    std::string CasadiKinDyn::computeGravity()
    {
        return impl().computeGravity();
    }

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

    void CasadiKinDyn::set_tau(const std::vector<double> &joint_currents)
    {
        _impl->set_tau(joint_currents);
    }

}