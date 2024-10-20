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
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>

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
#include "KinovaClient/specifications.hpp"

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
namespace sc = std::chrono;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 60 // Network timeout (seconds)

// float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
// float time_duration = DURATION; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

// Struct to hold feedback data
struct FeedbackData
{
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> currents;
    std::vector<double> torques;
};

// Struct to hold base command data
struct CommandData
{
    std::vector<double> basecommand;
};

// Shared queue for feedback data
std::mutex queueMutex;
std::queue<FeedbackData> feedbackQueue;
std::condition_variable queueCondition;

std::queue<CommandData> commandQueue;

// PID Controller class
class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double measured_value, double dt)
    {
        double error = setpoint - measured_value;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

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

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

bool move_to_home_position(k_api::Base::BaseClient *base)
{

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

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
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
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions());

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}

void move_to_retract_position(k_api::Base::BaseClient *base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
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

bool move_to_pref_position(k_api::Base::BaseClient *base)
{
    bool success = false;

    auto product = base->GetProductConfiguration();
    bool gen3LiteModelCompatible = false;
    if (product.model() == k_api::ProductConfiguration::MODEL_ID_L53 || product.model() == k_api::ProductConfiguration::MODEL_ID_L31)
    {
        if (product.model() == k_api::ProductConfiguration::MODEL_ID_L31)
        {
            gen3LiteModelCompatible = true; // Detected a Gen3 Lite
        }
    }
    else
    {
        std::cout << "Product is not compatible to run this example please contact support with KIN number bellow" << std::endl;
        std::cout << "Product KIN is : " << product.kin() << std::endl;
        return success;
    }

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();

    // Binded to degrees of movement and each degrees correspond to one degree of liberty
    auto actuators = base->GetActuatorCount();
    uint32_t degreesOfFreedom = actuators.count();

    // Move arm with waypoints list
    const int kmaxDegreesOfFreedom = 7;
    auto jointPoses = std::vector<std::array<float, kmaxDegreesOfFreedom>>();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Note : To customize this example for your needs an array of array is used containing the information needed :
    // all  values correspond to a joint/motor
    // If you have 6DoF the array will contain 6 positions expressed in degrees in float format.
    // If you have 7DoF the array will contain 7 positions expressed in degrees in float format.
    // You may overwrite the jointPose array for the proper arm
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (gen3LiteModelCompatible == true) // Gen3 Lite
    {
        // jointPoses.push_back({ 0.0f,  344.0f, 75.0f,  360.0f, 300.0f, 0.0f  });  // Home
        // jointPoses.push_back({ 0.0f,  21.0f,  145.0f, 272.0f, 32.0f,  273.0f}); // Retract
        // jointPoses.push_back({ 42.0f, 334.0f, 79.0f,  241.0f, 305.0f, 56.0f });// Angular pick down
        jointPoses.push_back({0.0f, 20.0f, 90.0f, 0.0f, 0.0f, 0.0f}); // Pref
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    const float firstTime = 5.0f;

    for (auto index = 0; index < jointPoses.size(); ++index)
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if (wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            k_api::Base::AngularWaypoint *ang = wpt->mutable_angular_waypoint();
            if (ang != nullptr)
            {
                for (auto angleIndex = 0; angleIndex < degreesOfFreedom; ++angleIndex)
                {
                    ang->add_angles(jointPoses.at(index).at(angleIndex));
                }

                // Joints/motors 5 and 7 are slower and need more time
                if (index == 4 || index == 6)
                {
                    ang->set_duration(firstTime * 6); // min 30 seconds
                }
                else
                {
                    ang->set_duration(firstTime);
                }
            }
        }
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart = base->OnNotificationActionTopic(create_event_listener_by_promise(finish_promise_cart),
                                                                            k_api::Common::NotificationOptions());

    k_api::Base::WaypointValidationReport result;
    try
    {
        // Verify validity of waypoints
        auto validationResult = base->ValidateWaypointList(wpts);
        result = validationResult;
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Try catch error on waypoint list" << std::endl;
        // You can print the error informations and error codes
        auto error_info = ex.getErrorInfo().getError();
        std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

        std::cout << "KError error_code: " << error_info.error_code() << std::endl;
        std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
        std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

        // Error codes by themselves are not very verbose if you don't see their corresponding enum value
        // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
        std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
        std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
        return false;
    }

    // Trajectory error report always exists and we need to make sure no elements are found in order to validate the trajectory
    if (result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << jointPoses.size() << " angular waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch (k_api::KDetailedException &ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // You can print the error informations and error codes
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            // Error codes by themselves are not very verbose if you don't see their corresponding enum value
            // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto ang_status = finish_future_cart.wait_for(std::chrono::seconds{100});

        base->Unsubscribe(promise_notification_handle_cart);

        if (ang_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for angular waypoint trajectory" << std::endl;
        }
        else
        {
            const auto ang_promise_event = finish_future_cart.get();
            std::cout << "Angular waypoint trajectory completed" << std::endl;
            std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(ang_promise_event) << std::endl;

            success = true;

            // We are now ready to reuse the validation output to test default trajectory generated...
            // Here we need to understand that trajectory using angular waypoint is never optimized.
            // In other words the waypoint list is the same and this is a limitation of Kortex API for now
        }
    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl;
        result.trajectory_error_report().PrintDebugString();
    }

    return success;
}

// std::vector<double> ratios = {1.0282, 0.3574, 1.0282, 1.9074, 2.0373, 1.9724};

// Function for dynamic calculations
void sendMobileCommands()
{
    // Create Mobile Base Serial
    std::string portName = "/dev/ttyACM0"; // Example port name
    speed_t baudRate = B115200;            // Example baud rate
    // Create an instance of KinovaMobileSerial
    KinovaMobile::KinovaMobileController mobile(portName);

    // Send velocities in a loop
    while (true)
    {

        // Retrieve feedback data from the queue
        CommandData base;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            // Wait until the queue is not empty
            queueCondition.wait(lock, []
                                { return !commandQueue.empty(); });
            base = commandQueue.front();
            commandQueue.pop();
        }

        std::cout << "Send BaseCommand : " << base.basecommand << std::endl;

        // Send velocities
        mobile.SendRefVelocities(static_cast<float>(base.basecommand[0]), static_cast<float>(base.basecommand[1]), static_cast<float>(base.basecommand[2]));

        // Wait for 5 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Mobile
    mobile.Move();
    mobile.SendRefVelocities(0, 0, 0);
    mobile.Stop();
    mobile.CloseInterface();
}

bool actuator_low_level_current_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient *actuator_config)
{
    bool return_status = true;

    // Kinova var
    std::vector<double> jntCmdTorques(6), jntPositions(6), jntVelocities(6), jntCurrents(6), jntTorque(6), jntImpedanceTorques(6);
    std::vector<double> TorqueGravity(6, 0.0), currentCommand(6, 0.0), currentGravityCommand(6, 0.0), currentFrictionCommand(6, 0.0);
    std::vector<double> ComTotalFrictionDir(6, 0.0), ComFrictionVelDir(6, 0.0), ComFrictionCurDir(6, 0.0), currentImpCommand(6, 0.0);
    std::vector<double> ComNullSpace(6, 0.0);

    std::vector<double> currentPos(3, 0.0);
    std::vector<double> errorPos(3, 0.0);
    std::vector<double> force(3, 0.0);

    std::vector<double> PrefPosition(3, 0.0);
    std::vector<double> TargetPosition(3, 0.0);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> result;

    // IK
    std::vector<double> JointPositionIK(6, 0.0);

    // Base var
    std::vector<double> BaseCommand(3, 0.0);

    std::vector<double> pidOutput(TorqueGravity.size()); // Vector to store PID controller output

    // Joint Specs

    // Create log file
    std::ofstream SaveData;
    SaveData.open("/home/rama/Documents/cpp/KinovaCompliance/data_gravity_vertical.csv");

    // Create Mobile Base Serial
    std::string portName = "/dev/ttyACM0"; // Example port name
    speed_t baudRate = B115200;            // Example baud rate
    // Create an instance of KinovaMobileSerial
    KinovaMobile::KinovaMobileController mobile(portName);

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();

    // // PID controllers for each joint
    // std::vector<PIDController> pidControllers;
    // for (size_t i = 0; i < TorqueGravity.size(); ++i)
    // {
    //     pidControllers.emplace_back(0.91, 20.0, 0.00002); // Replace with your PID gains
    // }

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

    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE.urdf");

    // Create an instance of CasadiKinDyn
    casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        try
        {
            servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base->SetServoingMode(servoing_mode);
            base_feedback = base_cyclic->RefreshFeedback();
        }
        catch (...)
        {
            std::cerr << "error setting low level control mode" << std::endl;
            throw;
        }

        // Initialize each actuator to their current position
        // Save the current actuator position, to avoid a following error
        for (int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // get current position for set q
        base_feedback = base_cyclic->RefreshFeedback();

        for (int i = 0; i < actuator_count; i++)
        {
            jntPositions[i] = base_feedback.actuators(i).position();              // deg
            jntVelocities[i] = DEG_TO_RAD(base_feedback.actuators(i).velocity()); // deg
            jntCurrents[i] = base_feedback.actuators(i).current_motor();
            jntTorque[i] = base_feedback.actuators(i).torque(); // nm
        }
        // Kinova API provides only positive angle values
        // This operation is required to align the logic with our safety monitor
        // We need to convert some angles to negative values
        // if (jntPositions[1] > DEG_TO_RAD(180.0))
        //     jntPositions[1] = jntPositions[1] - DEG_TO_RAD(360.0);
        // if (jntPositions[3] > DEG_TO_RAD(180.0))
        //     jntPositions[3] = jntPositions[3] - DEG_TO_RAD(360.0);
        // if (jntPositions[5] > DEG_TO_RAD(180.0))
        //     jntPositions[5] = jntPositions[5] - DEG_TO_RAD(360.0);

        // std::cout << "Before Pos : " << jntPositions << std::endl;
        // Adjust a position

        // Create Position instances
        std::vector<Position> positions;
        for (const auto &[key, value] : position)
        {
            positions.emplace_back("Joint" + std::to_string(key), value);
        }

        for (int i = 0; i < actuator_count; i++)
        {
            adjustPosition(i, jntPositions[i]);
            jntPositions[i] = DEG_TO_RAD(jntPositions[i]);
        }

        // std::cout << "After Pos : " << jntPositions << std::endl;
        // return true;

        kin_dyn.set_q(jntPositions); // Set the joint positions
        kin_dyn.set_prefTargetx();

        PrefPosition = kin_dyn.get_pos();
        TargetPosition = PrefPosition;

        // Set last actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);

        // Set to Current Mode
        for (int actuator_id = 1; actuator_id < actuator_count; actuator_id++)
            actuator_config->SetControlMode(control_mode_message, actuator_id);

        const std::vector<double> joint_currents_limits{9.2538, 2.7666, 9.2538, 17.1666, 18.3357, 17.7516};
        const std::vector<double> joint_position_upper_limits{159, 159, 159, 154, 154, 154};
        const std::vector<double> joint_position_lower_limits{-159, -159, -159, -154, -154, -154};

        // Real-time loop
        const int SECOND_IN_MICROSECONDS = 1000000;
        const int RATE_HZ = 600; // Hz
        const int TASK_TIME_LIMIT_SEC = 60;
        const sc::microseconds TASK_TIME_LIMIT_MICRO(TASK_TIME_LIMIT_SEC * SECOND_IN_MICROSECONDS);
        const sc::microseconds LOOP_DURATION(SECOND_IN_MICROSECONDS / RATE_HZ);

        int iterationCount = 0;
        int slowLoopCount = 0;
        sc::time_point<sc::steady_clock> controlStartTime = sc::steady_clock::now();
        sc::time_point<sc::steady_clock> loopStartTime = sc::steady_clock::now();
        sc::duration<int64_t, nano> totalElapsedTime = loopStartTime - controlStartTime;

        sc::time_point<sc::steady_clock> UpdateBaseCommandTime = sc::steady_clock::now();
        sc::duration<double, milli> BaseCommandTimeDelay = loopStartTime - UpdateBaseCommandTime;
        const sc::milliseconds COMMAND_BASE_TIME(5);

        // std::thread baseCommadThread(sendMobileCommands);

        ///////////////////////////////////////////////////////////////////////////////

        // Motion parameters
        double amplitude = 0.4; // Maximum distance of 1 meter
        double period = 8.0;   // Move 1 meter left in 2 seconds, then 1 meter right in the next 2 seconds
        double frequency = 0.5 / period;

        ///////////////////////////////////////////////////////////////////////////////

        while (totalElapsedTime < TASK_TIME_LIMIT_MICRO)
        {
            iterationCount++;
            loopStartTime = sc::steady_clock::now();
            totalElapsedTime = loopStartTime - controlStartTime;
            BaseCommandTimeDelay = loopStartTime - UpdateBaseCommandTime;

            base_feedback = base_cyclic->RefreshFeedback();

            for (int i = 0; i < actuator_count; i++)
            {
                jntPositions[i] = base_feedback.actuators(i).position();              // deg
                jntVelocities[i] = DEG_TO_RAD(base_feedback.actuators(i).velocity()); // deg
                jntCurrents[i] = base_feedback.actuators(i).current_motor();
                jntTorque[i] = base_feedback.actuators(i).torque(); // nm
            }

            // Kinova API provides only positive angle values
            // This operation is required to align the logic with our safety monitor
            // We need to convert some angles to negative values
            // if (jntPositions[1] > DEG_TO_RAD(180.0))
            //     jntPositions[1] = jntPositions[1] - DEG_TO_RAD(360.0);
            // if (jntPositions[3] > DEG_TO_RAD(180.0))
            //     jntPositions[3] = jntPositions[3] - DEG_TO_RAD(360.0);
            // if (jntPositions[5] > DEG_TO_RAD(180.0))
            //     jntPositions[5] = jntPositions[5] - DEG_TO_RAD(360.0);

            // std::cout << "Pos before: " << jntPositions << std::endl;

            // Create Position instances
            std::vector<Position> positions;
            for (const auto &[key, value] : position)
            {
                positions.emplace_back("Joint" + std::to_string(key), value);
            }

            // Adjust a position
            for (int i = 0; i < actuator_count; i++)
            {
                adjustPosition(i, jntPositions[i]);
                jntPositions[i] = DEG_TO_RAD(jntPositions[i]);
            }

            std::cout << "------------------------------------------------------" << std::endl;
            // std::cout << "Joint : " << actuator_count << std::endl;
            std::cout << "Pos : " << jntPositions << std::endl;
            std::cout << "Vel : " << jntVelocities << std::endl;
            std::cout << "Current : " << jntCurrents << std::endl;
            std::cout << "Torque : " << jntTorque << std::endl;
            // std::cout << "ratios : " << ratios << std::endl;
            // std::cout << "frictions : " << frictions << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;

            // Set current joint position and velocities
            kin_dyn.set_q(jntPositions);
            kin_dyn.set_qdot(jntVelocities);

            // Add friction compensation in the direction the joint is moving.
            ComFrictionVelDir = kin_dyn.compensateFrictionInMovingDirection();

            // Compute the gravity torque
            currentGravityCommand = kin_dyn.computeGravity();
            // // Compute desired current for gravity compensation
            // for (int i = 0; i < actuator_count; i++)
            // {
            //     currentGravityCommand[i] = (TorqueGravity[i] * ratios[i]);
            // }

            // Compensate friction in Velocity and Current direction
            kin_dyn.set_current(jntCurrents);
            ComTotalFrictionDir = kin_dyn.compensateFrictionInImpedanceMode(jntCurrents);

            // Return the current due to cartesian impedance
            // currentImpCommand = kin_dyn.cartesianImpedance();

            // std::pair<std::vector<double>, std::vector<double>> result = kin_dyn.cartesianImpedance();
            result = kin_dyn.cartesianImpedance();

            // // Retrieve error and current from the pair
            // errorPos = result.first;
            // currentImpCommand = result.second;

            // Retrieve error, current, and force from the tuple
            errorPos = std::get<0>(result);
            currentImpCommand = std::get<1>(result);
            force = std::get<2>(result);

            // Null Space
            // kin_dyn.set_q(jntPositions);
            // kin_dyn.set_qdot(jntVelocities);
            ComNullSpace = kin_dyn.NullSpaceTask();

            // std::vector<double>
            // Inverse Kinematics
            // kin_dyn.set_q(jntPositions);
            // JointPositionIK = kin_dyn.ComputeCLIK({0.377, 0.0, 0.61});

            // for (int i = 0; i < actuator_count; i++)
            // {
            //     JointPositionIK[i] = RAD_TO_DEG(JointPositionIK[i]);
            // }

            // Get EE Pos
            // kin_dyn.set_q(jntPositions);
            currentPos = kin_dyn.get_pos();

            ////////////////////////////////////////////////////////////////////////////////////

            double elapsed_seconds = sc::duration<double>(totalElapsedTime).count();

            // Generate smooth setpoint using sine wave for Y-axis motion
            TargetPosition[1] = amplitude * std::sin(2 * M_PI * frequency * elapsed_seconds);
            TargetPosition[0] = PrefPosition[0]; // Keep X constant
            TargetPosition[2] = PrefPosition[2] - 0.2; // Keep Z constant

            ////////////////////////////////////////////////////////////////////////////////////

            kin_dyn.set_targetPos({TargetPosition[0], TargetPosition[1], TargetPosition[2]});

            // Compute Total Current
            for (int i = 0; i < actuator_count; i++)
            {
                // currentCommand[i] = currentGravityCommand[i];
                // currentCommand[i] = currentGravityCommand[i] + ComFrictionVelDir[i];
                // currentCommand[i] = currentImpCommand[i] + currentGravityCommand[i];
                currentCommand[i] = currentGravityCommand[i] + ComTotalFrictionDir[i] + currentImpCommand[i];
                // currentCommand[i] = currentImpCommand[i] + currentGravityCommand[i] + ComTotalFrictionDir[i] + ComNullSpace[i];
                // currentCommand[i] = currentCommand[i] + ComNullSpace[i];
                // currentCommand[i] = ((currentImpCommand[i] + currentGravityCommand[i]) + ComTotalFrictionDir[i]) + ComNullSpace[i];
            }

            // // Apply PID control to reach the desired current
            // for (size_t i = 0; i < TorqueGravity.size(); ++i)
            // {
            //     pidOutput[i] = pidControllers[i].compute(currentCommand[i], jntCurrents[i], dt);
            // }

            // // Mobile base
            // BaseCommand = kin_dyn.CommandBase();

            std::cout << "------------------------------------------------------" << std::endl;
            // std::cout << "gravity : " << TorqueGravity << std::endl;
            std::cout << "currentGravityCommand : " << currentGravityCommand << std::endl;
            std::cout << "ComTotalFrictionDir : " << ComTotalFrictionDir << std::endl;
            std::cout << "currentImpCommand : " << currentImpCommand << std::endl;
            std::cout << "ComNullSpace : " << ComNullSpace << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "errorPos : " << errorPos << std::endl;
            std::cout << "force : " << force << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "BaseCommand : " << BaseCommand << std::endl;
            std::cout << "currentCommand : " << currentCommand << std::endl;
            // std::cout << "pidOutput : " << pidOutput << std::endl;
            std::cout << "JointPositionIK : " << JointPositionIK << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;

            // ------------------------------- save data -----------------------

            // SaveData << iterationCount << ","
            //          << (double)currentPos[0] << ","
            //          << (double)currentPos[1] << ","
            //          << (double)currentPos[2] << ","
            //          << (double)jntCurrents[0] << ","
            //          << (double)jntCurrents[1] << ","
            //          << (double)jntCurrents[2] << ","
            //          << (double)jntCurrents[3] << ","
            //          << (double)jntCurrents[4] << ","
            //          << (double)jntCurrents[5] << ","
            //          << (double)currentCommand[0] << ","
            //          << (double)currentCommand[1] << ","
            //          << (double)currentCommand[2] << ","
            //          << (double)currentCommand[3] << ","
            //          << (double)currentCommand[4] << ","
            //          << (double)currentCommand[5] << std::endl;

            // SaveData << iterationCount << ","
            //          << (double)errorPos[0] << ","
            //          << (double)errorPos[1] << ","
            //          << (double)-jntPositions[0] << ","
            //          << (double)BaseCommand[0] << ","
            //          << (double)BaseCommand[1] << ","
            //          << (double)BaseCommand[2] << std::endl;

            // -----------------------------------------------------------------

            // Set Current
            for (int i = 0; i < actuator_count; i++)
            {
                // pidOutput[i] = pidOutput[i];
                if (currentCommand[i] >= joint_currents_limits[i])
                    currentCommand[i] = joint_currents_limits[i] - 0.001;
                else if (currentCommand[i] <= -joint_currents_limits[i])
                    currentCommand[i] = -joint_currents_limits[i] + 0.001;
                base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                base_command.mutable_actuators(i)->set_current_motor(currentCommand[i]);
            }

            // // Set Position
            // for (int i = 0; i < actuator_count; i++)
            // {
            //     if (JointPositionIK[i] >= joint_position_upper_limits[i])
            //         JointPositionIK[i] = joint_position_upper_limits[i] - 0.001;
            //     else if (JointPositionIK[i] <= joint_position_lower_limits[i])
            //         JointPositionIK[i] = joint_position_lower_limits[i] + 0.001;
            //     base_command.mutable_actuators(i)->set_position(fmod(static_cast<float>(JointPositionIK[i]), 360.0f));
            // }

            // base_command.mutable_actuators(0)->set_position(fmod(0.0f, 360.0f));
            // base_command.mutable_actuators(1)->set_position(fmod(20.0f, 360.0f));
            // base_command.mutable_actuators(2)->set_position(fmod(90.0f, 360.0f));
            // base_command.mutable_actuators(3)->set_position(fmod(0.0f, 360.0f));
            // base_command.mutable_actuators(4)->set_position(fmod(0.0f, 360.0f));
            // base_command.mutable_actuators(5)->set_position(fmod(0.0f, 360.0f));

            // CommandData base;
            // for (int i = 0; i < 4; i++)
            // {
            //     base.basecommand.push_back(BaseCommand[i]);
            // }

            // // Mobile Base command send
            // if (BaseCommandTimeDelay > COMMAND_BASE_TIME)
            // {
            //     mobile.Move();
            //     mobile.SendRefVelocities(static_cast<float>(BaseCommand[0]), static_cast<float>(BaseCommand[1]), static_cast<float>(BaseCommand[2]));
            //     UpdateBaseCommandTime = sc::steady_clock::now();
            // }

            // Incrementing identifier ensures actuators can reject out of time frames
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535)
                base_command.set_frame_id(0);

            for (int idx = 0; idx < actuator_count; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException &ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;
                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (std::runtime_error &ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch (...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            // Enforce the constant loop time and count how many times the loop was late
            if (waitMicroSeconds(loopStartTime, LOOP_DURATION) != 0)
                slowLoopCount++;

            // break;
        }

        // // Mobile
        // mobile.Move();
        // mobile.SendRefVelocities(0, 0, 0);
        // mobile.Stop();
        // mobile.CloseInterface();

        // baseCommadThread.join();

        std::cout << "Completed" << std::endl;

        // Set first actuator back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

        for (int actuator_id = 1; actuator_id < 6; actuator_id++)
            actuator_config->SetControlMode(control_mode_message, actuator_id);

        std::cout << "Clean exit" << std::endl;

        // save log file
        SaveData.close();
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

///////////////////////////////////////////////////////////////////////
// class GripperCommandExample
//
// Implements a sample Gripper command application.
///////////////////////////////////////////////////////////////////////
class GripperCommandExample
{
public:
    GripperCommandExample(const std::string &ip_address, int port, const std::string &username = "admin", const std::string &password = "admin") : m_ip_address(ip_address), m_username(username), m_password(password), m_port(port)
    {
        m_router = nullptr;
        m_transport = nullptr;
        m_session_manager = nullptr;
        m_device_manager = nullptr;
        m_base = nullptr;

        m_is_init = false;
    }

    ~GripperCommandExample()
    {
        // Close API session
        m_session_manager->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        m_router->SetActivationStatus(false);
        m_transport->disconnect();

        // Destroy the API
        delete m_base;
        delete m_device_manager;
        delete m_session_manager;
        delete m_router;
        delete m_transport;
    }

    void Init()
    {
        if (m_is_init)
        {
            return;
        }
        m_transport = new k_api::TransportClientTcp();
        m_transport->connect(m_ip_address, m_port);
        m_router = new k_api::RouterClient(m_transport, [](k_api::KError err)
                                           { std::cout << "_________ callback error _________" << err.toString(); });

        // Set session data connection information
        auto createSessionInfo = k_api::Session::CreateSessionInfo();
        createSessionInfo.set_username(m_username);
        createSessionInfo.set_password(m_password);
        createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
        createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

        // Session manager service wrapper
        m_session_manager = new k_api::SessionManager(m_router);
        m_session_manager->CreateSession(createSessionInfo);

        // Create services
        m_device_manager = new k_api::DeviceManager::DeviceManagerClient(m_router);
        m_base = new k_api::Base::BaseClient(m_router);

        m_is_init = true;
    }

    bool Run()
    {
        if (m_is_init == false)
        {
            return false;
        }
        std::cout << "Performing gripper test in position..." << std::endl;

        k_api::Base::GripperCommand gripper_command;

        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);

        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        for (float position = 0.0; position < 1.0; position += 0.1)
        {
            std::cout << "Setting position to " << position << std::endl;
            finger->set_value(position);
            m_base->SendGripperCommand(gripper_command);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        return true;
    }

private:
    k_api::RouterClient *m_router;
    k_api::TransportClientTcp *m_transport;
    k_api::SessionManager *m_session_manager;
    k_api::Base::BaseClient *m_base;
    k_api::DeviceManager::DeviceManagerClient *m_device_manager;
    bool m_is_init;
    std::string m_username;
    std::string m_password;
    std::string m_ip_address;
    int m_port;
};

int main(int argc, char **argv)
{
    // Kinova
    auto parsed_args = ParseExampleArguments(argc, argv);

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

    // gripper
    GripperCommandExample *gripper_command_example;
    gripper_command_example = new GripperCommandExample(parsed_args.ip_address, PORT, parsed_args.username, parsed_args.password);
    gripper_command_example->Init();
    gripper_command_example->Run();

    delete gripper_command_example;

    // Example core
    bool success = true;
    success &= move_to_home_position(base);
    success &= move_to_pref_position(base);
    success &= actuator_low_level_current_control(base, base_cyclic, actuator_config);
    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
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

    return success ? 0 : 1;
}

// int main(int argc, char **argv)
// {
// // Create Mobile Base Serial
// std::string portName = "/dev/ttyACM0"; // Example port name
// speed_t baudRate = B115200;            // Example baud rate
// // Create an instance of KinovaMobileSerial
// KinovaMobile::KinovaMobileController mobile(portName);

// // mobile.SendRefPose(0, 0, 0);
// // mobile.Move();
// // mobile.Stop();
// // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

// const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE_custom.urdf");
// casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

// // std::vector<double> command = kin_dyn.CommandBase();

// while (1)
// {
//     // mobile.Move();
//     // mobile.SendRefVelocities(0.1, 0, 0);
//     // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//     // mobile.Stop();

//     // mobile.Move();
//     // mobile.SendRefVelocities(1, 0, 0);
//     // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//     // mobile.Stop();

//     // mobile.Move();
//     // mobile.SendRefVelocities(0, 0, 0);
//     // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//     // mobile.Stop();

//     // for (float i = 0.1; i <= 1.0; i += 0.1) // Increment by 0.1
//     // {
//     //     mobile.Move();
//     //     mobile.SendRefVelocities(i, 0, 0);
//     //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     //     // mobile.Stop();
//     // }

//     for (float i = 0.0; i <= 0.5; i += 0.001) // Increment by 0.1
//     {
//         mobile.Move();
//         mobile.SendRefVelocities(i, 0, 0);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         // mobile.Stop();
//     }

//     for (float i = 0.5; i >= 0.0; i -= 0.001) // Increment by 0.1
//     {
//         mobile.Move();
//         mobile.SendRefVelocities(i, 0, 0);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         // mobile.Stop();
//     }

//     for (float i = 0.0; i <= 0.5; i += 0.001) // Increment by 0.1
//     {
//         mobile.Move();
//         mobile.SendRefVelocities(0, 0, i);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         // mobile.Stop();
//     }

//     for (float i = 0.5; i >= 0.0; i -= 0.001) // Increment by 0.1
//     {
//         mobile.Move();
//         mobile.SendRefVelocities(0, 0, i);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         // mobile.Stop();
//     }
// }

// mobile.Stop();
// mobile.CloseInterface();

// return 0;
// }

// int main(int argc, char **argv)
// {
//     // // Variables
//     // SX x = SX::sym("x", 2, 2);
//     // SX y = SX::sym("y");

//     // // Simple function
//     // Function f("f", {x, y}, {sqrt(y) - 1, sin(x) - y});

//     // // Generate C-code
//     // f.generate("f");

//     // // Compile the C-code to a shared library
//     // std::string compile_command = "gcc -fPIC -shared -O3 f.c -o f.so";
//     // int flag = system(compile_command.c_str());
//     // casadi_assert(flag == 0, "Compilation failed");

//     // // Usage from C++
//     // usage_cplusplus();

//     // // List of C files to compile
//     // std::vector<std::string> files = {
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/dq.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/dx.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/F.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/g.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/J.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/JT.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/lam.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/mu.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/N.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/Nv.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/T.c",
//     //     "/home/rama/Documents/cpp/KinovaCompliance/src/KinovaSymbolics/x.c"
//     // };

//     // // Output directory for .so files
//     // std::string output_dir = "/home/rama/Documents/cpp/KinovaCompliance/build/";

//     // // Compile each C file into a shared library
//     // for (const auto& file : files) {
//     //     // Extract the file name
//     //     size_t pos = file.find_last_of('/');
//     //     std::string filename = file.substr(pos + 1);

//     //     // Remove the .c extension from the file name
//     //     filename = filename.substr(0, filename.size() - 2); // remove last two characters (.c)

//     //     // Formulate the output path for the .so file
//     //     std::string output_path = output_dir + filename + ".so";

//     //     // Compile command
//     //     std::string compile_command = "gcc -fPIC -shared -O3 " + file + " -o " + output_path;

//     //     // Execute compilation command
//     //     int flag = system(compile_command.c_str());

//     //     // Check if compilation was successful
//     //     if (flag == 0) {
//     //         std::cout << "Compilation successful for " << file << std::endl;
//     //     } else {
//     //         std::cout << "Compilation failed for " << file << std::endl;
//     //     }
//     // }

//     // Initialize a vector of doubles with 5 zeros and append the scalar 6
//     std::vector<double> q = {0, 20, 90, 0, 0, 0};

//     //////////////////////////////////////////////////////////////
//     auto start = high_resolution_clock::now();

//     // Use CasADi's "external" to load the compiled function
//     Function g = external("g");

//     // Create a DM object from the modified vector q
//     DM input = DM(q);
//     // Create a vector with the single input
//     std::vector<DM> arg = {input};
//     // Call function g with the single input
//     std::vector<DM> res = g(arg);

//     std::cout << "result G(0): " << res.at(0) << std::endl;

//     auto stop = high_resolution_clock::now();
//     auto duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

//     //////////////////////////////////////////////////////////////

//     //////////////////////////////////////////////////////////////
//     std::vector<double> TorqueGravity(6, 0.0);

//     // You should change here to set up your own URDF file or just pass it as an argument of this example.
//     const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE_custom.urdf");
//     // Create an instance of CasadiKinDyn
//     casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

//     start = high_resolution_clock::now();

//     kin_dyn.set_q(q);
//     TorqueGravity = kin_dyn.computeGravity();

//     std::cout << "result G(1): " << TorqueGravity << std::endl;

//     stop = high_resolution_clock::now();
//     duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

//     //////////////////////////////////////////////////////////////

//     //////////////////////////////////////////////////////////////
//     start = high_resolution_clock::now();

//     Function J = external("J");

//     // Create a vector with the single input
//     std::vector<DM> arg_j = {input};
//     std::vector<DM> res_j = J(arg_j);

//     std::cout << "result J(0): " << res_j.at(0) << std::endl;

//     stop = high_resolution_clock::now();
//     duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;
//     //////////////////////////////////////////////////////////////

//     //////////////////////////////////////////////////////////////

//     start = high_resolution_clock::now();

//     std::string ss = kin_dyn.jacobian("END_EFFECTOR", casadi_kin_dyn::CasadiKinDyn::ReferenceFrame::LOCAL_WORLD_ALIGNED);

//     std::cout << "result J(1): " << ss << std::endl;

//     stop = high_resolution_clock::now();
//     duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;
//     //////////////////////////////////////////////////////////////

//     return 0;
// }