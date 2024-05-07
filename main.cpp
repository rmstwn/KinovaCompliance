// #include <CppLinuxSerial/SerialPort.hpp>

// using namespace mn::CppLinuxSerial;

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <cstdint>
#include <string>
#include <iostream>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

// Mobile base Lib
#include "KinovaMobile/kinova_mobile.hpp"

// Kinova API Lib
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include "KinovaClient/utilities.h"

// Pinocchio Casadi Lib
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

#include "casadi/casadi.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

#include "KinovaPinocchioCasadi/casadi_kin_dyn.h"

// Kinova Control Lib

#include "KinovaControl/state.hpp"

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 60 // Network timeout (seconds)

float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)

// Struct to hold feedback data
struct FeedbackData
{
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> currents;
    std::vector<double> torques;
};

// Shared queue for feedback data
std::mutex queueMutex;
std::queue<FeedbackData> feedbackQueue;
std::condition_variable queueCondition;

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

/**************************
 * Example core functions *
 **************************/
// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)>
check_for_end_or_abort(bool &finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch (notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

void move_to_home_position(k_api::Base::BaseClient *base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    }
    else
    {
        bool action_finished = false;
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options);

        base->ExecuteActionFromReference(action_handle);

        while (!action_finished)
        {
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

void move_to_retract_position(k_api::Base::BaseClient *base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Retract")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    }
    else
    {
        bool action_finished = false;
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options);

        base->ExecuteActionFromReference(action_handle);

        while (!action_finished)
        {
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

bool actuator_low_level_velocity_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic)
{
    bool return_status = true;

    // Move arm to ready position
    move_to_home_position(base);

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            // google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            std::cout << serialized_data << std::endl
                      << std::endl;
        };

        // Real-time loop
        while (timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if (now - last > 1000)
            {
                for (int i = 0; i < actuator_count; i++)
                {
                    // Move only the last actuator to prevent collision
                    if (i == actuator_count - 1)
                    {
                        commands[i] += (0.001f * velocity);
                        // base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                        // base_command.mutable_actuators(i)->set_torque_joint(1.0);
                    }

                    // base_command.mutable_actuators(i)->set_current_motor(0.0f);
                }

                try
                {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                    // Initialize each actuator to its current position
                    FeedbackData data;
                    for (int i = 0; i < actuator_count; i++)
                    {
                        data.positions.push_back(base_feedback.actuators(i).position());
                        data.velocities.push_back(base_feedback.actuators(i).velocity());
                        data.currents.push_back(base_feedback.actuators(i).current_motor());
                        data.torques.push_back(base_feedback.actuators(i).torque());
                    }

                    // Lock the queue and push the feedback data
                    {
                        std::lock_guard<std::mutex> lock(queueMutex);
                        feedbackQueue.push(data);
                    }
                    // Notify the condition variable to wake up the other thread
                    queueCondition.notify_one();
                }
                catch (...)
                {
                    timeout++;
                }

                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

std::vector<double> multiplyVectors(const std::vector<double> &vec1, const std::vector<double> &vec2)
{
    std::vector<double> result(vec1.size());
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), result.begin(), std::multiplies<double>());
    return result;
}

// Overload the += operator for std::vector<float>
std::vector<double> &operator+=(std::vector<double> &lhs, const std::vector<double> &rhs)
{
    // Ensure both vectors have the same size
    if (lhs.size() != rhs.size())
    {
        throw std::invalid_argument("Vectors have different sizes");
    }

    // Perform element-wise addition
    std::transform(lhs.begin(), lhs.end(), rhs.begin(), lhs.begin(), std::plus<double>());

    return lhs;
}

std::vector<double> ratios = {1.0282, 0.3074, 1.0282, 1.9074, 2.0373, 1.9724};

bool actuator_low_level_current_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient *actuator_config)
{
    bool return_status = true;

    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE_custom.urdf");

    // Create an instance of CasadiKinDyn
    casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

    // Move arm to ready position
    move_to_home_position(base);

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    // Calculate the element-wise product of self.state.g and self.state.ratios
    std::vector<double> gravityCompensation;

    std::vector<double> currentCommand(6, 0.0);

    std::cout << "Initializing the arm for current low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Clearing faults
        try
        {
            base->ClearFaults();
        }
        catch (...)
        {
            std::cout << "Unable to clear robot faults" << std::endl;
            return false;
        }

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);

        for (int i = 1; i < actuator_count; i++)
        {
            actuator_config->SetControlMode(control_mode_message, i);
        }

        // Retrieve feedback data from the queue
        FeedbackData data;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            // Wait until the queue is not empty
            queueCondition.wait(lock, []
                                { return !feedbackQueue.empty(); });
            data = feedbackQueue.front();
            feedbackQueue.pop();
        }

        // Convert positions from degrees to radians
        std::vector<double> positionsInRadians;
        for (const auto &pos_deg : data.positions)
        {
            positionsInRadians.push_back(pos_deg * (M_PI / 180.0));
        }

        kin_dyn.set_q(positionsInRadians); // Set the joint positions

        // int actuator_device_id = 2;
        // actuator_config->SetControlMode(control_mode_message, actuator_device_id);

        // Real-time loop
        while (timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if (now - last > 1000)
            {

                std::cout << "Current : " << data.currents << std::endl;

                std::cout << "Pos : " << positionsInRadians << std::endl;

                std::vector<double> gravity = kin_dyn.computeGravity();
                std::cout << "gravity : " << gravity << std::endl;

                std::cout << "ratios : " << ratios << std::endl;

                // // Print sizes of gravity and ratios
                // std::cout << "Size of gravity: " << gravity.size() << std::endl;
                // std::cout << "Size of ratios: " << ratios.size() << std::endl;

                // // Calculate the element-wise product of gravity and ratios
                // gravityCompensation.reserve(gravity.size()); // Reserve space to avoid reallocations

                // // Perform element-wise multiplication
                // std::transform(gravity.begin(), gravity.end(), ratios.begin(), std::back_inserter(gravityCompensation),
                //                std::multiplies<double>());

                // // Add the element-wise product to current
                // std::cout << "Size of current: " << currentCommand.size() << std::endl;
                // currentCommand += gravityCompensation;

                // std::cout << "Current : " << gravity << std::endl;

                // Verify sizes of gravity and ratios
                if (gravity.size() != ratios.size())
                {
                    throw std::invalid_argument("Gravity and ratios vectors have different sizes");
                }

                // Initialize result vector with the size of current
                std::vector<double> result(currentCommand.size(), 0.0);

                // Perform element-wise addition manually
                for (size_t i = 0; i < currentCommand.size(); ++i)
                {
                    // Convert scalar values to vectors
                    std::vector<double> gravityScalar = {gravity[i]};
                    std::vector<double> ratiosScalar = {ratios[i]};

                    // Perform addition
                    result[i] += currentCommand[i] + (gravityScalar[0] * ratiosScalar[0]);
                }

                std::cout << "result : " << result << std::endl;

                // base_command.mutable_actuators(1)->set_current_motor(result[1]);

                for (int i = 0; i < actuator_count; i++)
                {
                    base_command.mutable_actuators(i)->set_current_motor(result[i]);
                }

                // for (int i = 0; i < actuator_count; i++)
                // {
                //     // // Move only the last actuator to prevent collision
                //     if (i == actuator_count - 1)
                //     {
                //         // commands[i] += (0.001f * velocity);
                //         // base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                //         // base_command.mutable_actuators(i)->set_torque_joint(1.0);
                //         base_command.mutable_actuators(i)->set_current_motor(0.0f);
                //     }
                // }

                // base_command.mutable_actuators(3)->set_current_motor(0.1f);
                // base_command.mutable_actuators(4)->set_current_motor(0.1f);
                // base_command.mutable_actuators(0)->set_current_motor(0.3f);

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                    base_cyclic->Refresh_callback(base_command, 0);
                }
                catch (...)
                {
                    timeout++;
                }

                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

// Function to receive Kinova feedback data
void receiveFeedbackData(ExampleArgs parsed_args)
{
    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        // return_status = false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    bool return_status = true;

    while (return_status)
    {
        try
        {
            base_feedback = base_cyclic->RefreshFeedback();

            int actuator_count = base->GetActuatorCount().count();

            // Initialize each actuator to its current position
            FeedbackData data;
            for (int i = 0; i < actuator_count; i++)
            {
                data.positions.push_back(base_feedback.actuators(i).position());
                data.velocities.push_back(base_feedback.actuators(i).velocity());
                data.currents.push_back(base_feedback.actuators(i).current_motor());
                data.torques.push_back(base_feedback.actuators(i).torque());
            }

            // std::cout << "Received Position:" << std::endl;
            // for (const auto &pos : data.positions)
            // {
            //     std::cout << pos << std::endl;
            // }
            // std::cout << "Received Velocity:" << std::endl;
            // for (const auto &vel : data.velocities)
            // {
            //     std::cout << vel << std::endl;
            // }
            // std::cout << "Received Current:" << std::endl;
            // for (const auto &curr : data.currents)
            // {
            //     std::cout << curr << std::endl;
            // }
            // std::cout << "Received Torque:" << std::endl;
            // for (const auto &curr : data.torques)
            // {
            //     std::cout << curr << std::endl;
            // }

            // Lock the queue and push the feedback data
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                feedbackQueue.push(data);
            }
            // Notify the condition variable to wake up the other thread
            queueCondition.notify_one();

            // Sleep for a short duration (simulate Kinova feedback rate)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        catch (k_api::KDetailedException &ex)
        {
            std::cout << "Kortex error: " << ex.what() << std::endl;
            return_status = false;
        }
        catch (std::runtime_error &ex2)
        {
            std::cout << "Runtime error: " << ex2.what() << std::endl;
            return_status = false;
        }
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}

// Function for dynamic calculations
void performDynamicCalculations(ExampleArgs parsed_args)
{
    // using namespace pinocchio;

    // // You should change here to set up your own URDF file or just pass it as an argument of this example.
    // const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE_custom.urdf");

    // // Create an instance of CasadiKinDyn
    // casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    bool return_status = true;

    // Example core
    auto isOk = actuator_low_level_current_control(base, base_cyclic, actuator_config);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    // while (true)
    // {
    //     // Retrieve feedback data from the queue
    //     FeedbackData data;
    //     {
    //         std::unique_lock<std::mutex> lock(queueMutex);
    //         // Wait until the queue is not empty
    //         queueCondition.wait(lock, []
    //                             { return !feedbackQueue.empty(); });
    //         data = feedbackQueue.front();
    //         feedbackQueue.pop();
    //     }

    //     // // Perform dynamic calculations using the received feedback data
    //     // // Print each element of the vectors
    //     // std::cout << "Dynamic Calculations:" << std::endl;

    //     std::cout << "Received Position:" << std::endl;
    //     for (const auto &pos : data.positions)
    //     {
    //         std::cout << pos << std::endl;
    //     }
    //     std::cout << "Received Velocity:" << std::endl;
    //     for (const auto &vel : data.velocities)
    //     {
    //         std::cout << vel << std::endl;
    //     }
    //     std::cout << "Received Current:" << std::endl;
    //     for (const auto &curr : data.currents)
    //     {
    //         std::cout << curr << std::endl;
    //     }
    //     std::cout << "Received Torque:" << std::endl;
    //     for (const auto &curr : data.torques)
    //     {
    //         std::cout << curr << std::endl;
    //     }

    //     // Convert positions from degrees to radians
    //     std::vector<double> positionsInRadians;
    //     for (const auto &pos_deg : data.positions)
    //     {
    //         positionsInRadians.push_back(pos_deg * (M_PI / 180.0));
    //     }

    //     kin_dyn.set_q(positionsInRadians); // Set the joint positions

    //     // // Call the fk function with the desired link name
    //     // std::string link_name = "END_EFFECTOR";
    //     // std::string fk_result = kin_dyn.fk(link_name);

    //     // // Print the result
    //     // std::cout << "FK Result for link " << link_name << ":\n"
    //     //           << fk_result << std::endl;

    //     // // Call the Jacoboian function with the desired link name and ref
    //     // casadi_kin_dyn::CasadiKinDyn::ReferenceFrame ref = casadi_kin_dyn::CasadiKinDyn::ReferenceFrame::LOCAL;
    //     // std::string jacobian_result = kin_dyn.jacobian(link_name, ref);

    //     // // Print or use the result as needed
    //     // std::cout << "Result of jacobian function:" << std::endl;
    //     // std::cout << jacobian_result << std::endl;

    //     // Compute Gravity vector
    //     std::string gravity = kin_dyn.computeGravity();
    //     std::cout << "Result of Gravity function:" << std::endl;
    //     std::cout << gravity << std::endl;

    //     // Sleep for a short duration (simulate dynamic calculation rate)
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
}

int main(int argc, char **argv)
{
    // auto parsed_args = ParseExampleArguments(argc, argv);

    // Parse command line arguments
    ExampleArgs parsed_args = ParseExampleArguments(argc, argv);

    // Create two exclusive threads: one for receiving feedback data and another for dynamic calculations
    std::thread feedbackThread(receiveFeedbackData, parsed_args);
    std::thread calculationsThread(performDynamicCalculations, parsed_args);

    // Join the threads to wait for them to finish (though they won't, as they are infinite loops)
    feedbackThread.join();
    calculationsThread.join();

    return 0;
}
