#ifndef KINOVA_CLIENT_HPP
#define KINOVA_CLIENT_HPP

#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

// Kinova Lib
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <InterconnectConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "specifications.hpp"
#include "state.hpp"
#include "KinovaControl/rate_counter.hpp"

namespace k_api = Kinova::Api;

namespace kinova
{
    class KinovaClient
    {

    public:
        KinovaClient(State *state, k_api::Base::BaseClient *base = nullptr, k_api::BaseCyclic::BaseCyclicClient *base_cyclic = nullptr,
                     k_api::ActuatorConfig::ActuatorConfigClient *actuator_config = nullptr, k_api::RouterClient *router = nullptr,
                     k_api::RouterClient *real_time_router = nullptr, bool simulate = false);
        ~KinovaClient();

        std::string get_mode();
        std::vector<std::string> get_control_modes();
        std::string get_servoing_mode();
        void clear_faults();
        void start_in_new_thread();
        void start();
        void stop();
        void set_control_mode(int joint, std::string mode);
        void start_HLT();
        void stop_HLT();
        void start_LLC();
        void stop_LLC();
        void connect_LLC();
        void disconnect_LLC();
        bool home();
        bool zero();
        bool retract();
        bool pref();
        void high_level_move(Position position);
        float get_position(int joint, bool as_percentage);
        float get_velocity(int joint, bool as_percentage);
        float get_current(int joint, bool as_percentage);
        float get_gripper_position();
        float get_torque(int joint, bool as_percentage);
        void copy_feedback_to_command(int joint);
        void set_command(std::vector<float> commands);
        void toggle_active(int joint);
        void update_state();

    private:
        State *state;
        k_api::Base::BaseClient *base;
        k_api::BaseCyclic::BaseCyclicClient *base_cyclic;
        k_api::ActuatorConfig::ActuatorConfigClient *actuator_config;
        k_api::RouterClient *router;
        k_api::RouterClient *real_time_router;
        bool simulate;
        int actuator_count;
        std::vector<bool> joint_active;
        bool calibrating;
        bool changing_servoing_mode;
        bool controller_connected;
        bool active;
        RateCounter rate_counter;
        std::string mode;
        k_api::Base::ServoingModeInformation servoing_mode;
        k_api::ActuatorConfig::ControlMode actuator_modes;
        k_api::BaseCyclic::Command command;
        k_api::BaseCyclic::Feedback feedback;

        // std::vector<ActuatorConfig_pb2::ControlMode> actuator_modes;
        // BaseCyclic_pb2::Command command;
        // BaseCyclic_pb2::Feedback feedback;

        std::mutex mtx;
        std::condition_variable cv;

        void _refresh_loop();
        void _refresh();
        void _set_servoing_mode(k_api::Base::ServoingMode value);
        void _update_modes();
        void _initialize_command();
        void _high_level_move(Position position);
        void _move_gripper(bool close);
        void _high_level_tracking();
        bool _execute_action(k_api::Base::Action action);
        std::function<void(k_api::Base::ActionNotification)> _check_for_end_or_abort();
    };
}

#endif // KINOVA_CLIENT_HPP