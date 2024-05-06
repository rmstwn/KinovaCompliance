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

// Kinova Lib
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
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

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

// Struct to hold feedback data
struct FeedbackData
{
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> currents;
};

// Shared queue for feedback data
std::mutex queueMutex;
std::queue<FeedbackData> feedbackQueue;
std::condition_variable queueCondition;

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
            }

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
    using namespace pinocchio;

    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/GEN3-LITE_custom.urdf");

    // Create an instance of CasadiKinDyn
    casadi_kin_dyn::CasadiKinDyn kin_dyn(urdf_filename);

    while (true)
    {
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

        // // Perform dynamic calculations using the received feedback data
        // // Print each element of the vectors
        // std::cout << "Dynamic Calculations:" << std::endl;
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

        std::cout << "Received Position:" << std::endl;
        for (const auto &pos : data.positions)
        {
            std::cout << pos << std::endl;
        }

        // Convert positions from degrees to radians
        std::vector<double> positionsInRadians;
        for (const auto &pos_deg : data.positions)
        {
            positionsInRadians.push_back(pos_deg * (M_PI / 180.0));
        }

        kin_dyn.setJointPositions(positionsInRadians); // Set the joint positions

        // Call the fk function with the desired link name
        std::string link_name = "END_EFFECTOR";
        std::string fk_result = kin_dyn.fk(link_name);

        // Print the result
        std::cout << "FK Result for link " << link_name << ":\n"
                  << fk_result << std::endl;

        // Call the Jacoboian function with the desired link name and ref
        casadi_kin_dyn::CasadiKinDyn::ReferenceFrame ref = casadi_kin_dyn::CasadiKinDyn::ReferenceFrame::LOCAL;
        std::string jacobian_result = kin_dyn.jacobian(link_name, ref);

        // Print or use the result as needed
        std::cout << "Result of jacobian function:" << std::endl;
        std::cout << jacobian_result << std::endl;

        casadi::

        // Sleep for a short duration (simulate dynamic calculation rate)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
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
