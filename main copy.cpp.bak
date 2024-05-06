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

#include <thread>
#include <iostream>
#include <chrono>
#include <mutex>
#include <queue>
#include <condition_variable>

#include "KinovaClient/utilities.h"

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

// void example_api_creation(int argc, char **argv)
// {
//     auto parsed_args = ParseExampleArguments(argc, argv);

//     // -----------------------------------------------------------
//     // How to create an API with the SessionManager, DeviceConfigClient and BaseClient services
//     auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
//     auto transport = new k_api::TransportClientTcp();
//     auto router = new k_api::RouterClient(transport, error_callback);
//     transport->connect(parsed_args.ip_address, PORT);

//     // Set session data connection information
//     auto create_session_info = k_api::Session::CreateSessionInfo();
//     create_session_info.set_username(parsed_args.username);
//     create_session_info.set_password(parsed_args.password);
//     create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
//     create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

//     // Session manager service wrapper
//     std::cout << "Creating session for communication" << std::endl;
//     auto session_manager = new k_api::SessionManager(router);
//     session_manager->CreateSession(create_session_info);
//     std::cout << "Session created" << std::endl;

//     // Create DeviceConfigClient and BaseClient
//     auto device_config = new k_api::DeviceConfig::DeviceConfigClient(router);
//     auto base = new k_api::Base::BaseClient(router);

//     // -----------------------------------------------------------
//     // Now you're ready to use the API
//     // ...

//     // -----------------------------------------------------------
//     // After you're done, here's how to tear down the API

//     // Close API session
//     session_manager->CloseSession();

//     // Deactivate the router and cleanly disconnect from the transport object
//     router->SetActivationStatus(false);
//     transport->disconnect();

//     // Destroy the API
//     delete base;
//     delete device_config;
//     delete session_manager;
//     delete router;
//     delete transport;
// }

// int main(int argc, char **argv)
// {
//     // // Specify the port name and baud rate
//     // std::string portName = "/dev/ttyACM0"; // Example port name
//     // // std::string portName = "/tmp/ttyV0"; // Example port name

//     // speed_t baudRate = B115200; // Example baud rate

//     // // Create an instance of KinovaMobileSerial
//     // KinovaMobile::KinovaMobileController serial(portName);

//     // // Perform other operations with the serial port as needed

//     // // serial.ResetOdometry();
//     // serial.SendRefPose(1, 0, 0);

//     // serial.Move();

//     example_api_creation(argc, argv);

//     // return 0;
// }

// class GripperCommandExample
// {
// public:
//     GripperCommandExample(const std::string &ip_address, int port, const std::string &username = "admin", const std::string &password = "admin") : m_ip_address(ip_address), m_username(username), m_password(password), m_port(port)
//     {
//         m_router = nullptr;
//         m_transport = nullptr;
//         m_session_manager = nullptr;
//         m_device_manager = nullptr;
//         m_base = nullptr;

//         m_is_init = false;
//     }

//     ~GripperCommandExample()
//     {
//         // Close API session
//         m_session_manager->CloseSession();

//         // Deactivate the router and cleanly disconnect from the transport object
//         m_router->SetActivationStatus(false);
//         m_transport->disconnect();

//         // Destroy the API
//         delete m_base;
//         delete m_device_manager;
//         delete m_session_manager;
//         delete m_router;
//         delete m_transport;
//     }

//     void Init()
//     {
//         if (m_is_init)
//         {
//             return;
//         }
//         m_transport = new k_api::TransportClientTcp();
//         m_transport->connect(m_ip_address, m_port);
//         m_router = new k_api::RouterClient(m_transport, [](k_api::KError err)
//                                            { std::cout << "_________ callback error _________" << err.toString(); });

//         // Set session data connection information
//         auto createSessionInfo = k_api::Session::CreateSessionInfo();
//         createSessionInfo.set_username(m_username);
//         createSessionInfo.set_password(m_password);
//         createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
//         createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

//         // Session manager service wrapper
//         m_session_manager = new k_api::SessionManager(m_router);
//         m_session_manager->CreateSession(createSessionInfo);

//         // Create services
//         m_device_manager = new k_api::DeviceManager::DeviceManagerClient(m_router);
//         m_base = new k_api::Base::BaseClient(m_router);

//         m_is_init = true;
//     }

//     bool Run()
//     {
//         if (m_is_init == false)
//         {
//             return false;
//         }
//         std::cout << "Performing gripper test in position..." << std::endl;

//         k_api::Base::GripperCommand gripper_command;

//         gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);

//         auto finger = gripper_command.mutable_gripper()->add_finger();
//         finger->set_finger_identifier(1);
//         for (float position = 0.0; position < 1.0; position += 0.1)
//         {
//             std::cout << "Setting position to " << position << std::endl;
//             finger->set_value(position);
//             m_base->SendGripperCommand(gripper_command);
//             std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//         }

//         std::cout << "Opening gripper using speed command..." << std::endl;
//         gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
//         finger->set_value(0.1);
//         m_base->SendGripperCommand(gripper_command);
//         k_api::Base::Gripper gripper_feedback;
//         k_api::Base::GripperRequest gripper_request;
//         bool is_motion_completed = false;
//         gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
//         while (!is_motion_completed)
//         {
//             float position = 0.0f;
//             gripper_feedback = m_base->GetMeasuredGripperMovement(gripper_request);

//             if (gripper_feedback.finger_size())
//             {
//                 position = gripper_feedback.finger(0).value();
//                 cout << "Reported position : " << position << std::endl;
//             }

//             if (position < 0.01f)
//             {
//                 is_motion_completed = true;
//             }
//         }

//         std::cout << "Closing gripper using speed command..." << std::endl;
//         finger->set_value(-0.1);
//         m_base->SendGripperCommand(gripper_command);
//         is_motion_completed = false;
//         gripper_request.set_mode(k_api::Base::GRIPPER_SPEED);
//         while (!is_motion_completed)
//         {
//             float speed = 0.0;
//             gripper_feedback = m_base->GetMeasuredGripperMovement(gripper_request);
//             if (gripper_feedback.finger_size())
//             {
//                 speed = gripper_feedback.finger(0).value();
//                 cout << "Reported speed : " << speed << std::endl;
//             }

//             if (speed == 0.0f)
//             {
//                 is_motion_completed = true;
//             }
//         }
//         return true;
//     }

// private:
//     k_api::RouterClient *m_router;
//     k_api::TransportClientTcp *m_transport;
//     k_api::SessionManager *m_session_manager;
//     k_api::Base::BaseClient *m_base;
//     k_api::DeviceManager::DeviceManagerClient *m_device_manager;
//     bool m_is_init;
//     std::string m_username;
//     std::string m_password;
//     std::string m_ip_address;
//     int m_port;
// };

// int main(int argc, char **argv)
// {
//     auto parsed_args = ParseExampleArguments(argc, argv);

//     GripperCommandExample *gripper_command_example;
//     gripper_command_example = new GripperCommandExample(parsed_args.ip_address, PORT, parsed_args.username, parsed_args.password);
//     gripper_command_example->Init();
//     gripper_command_example->Run();
//     delete gripper_command_example;

//     return 0;
// };

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

    // while (true)
    // {
    //     // Simulate receiving position, velocity, and current feedback data from Kinova API
    //     // Here, we'll just increment values for demonstration
    //     FeedbackData data;
    //     data.position += 0.1;
    //     data.velocity += 0.2;
    //     data.current += 0.5;

    //     // Lock the queue and push the feedback data
    //     {
    //         std::lock_guard<std::mutex> lock(queueMutex);
    //         feedbackQueue.push(data);
    //     }

    //     // Notify the condition variable to wake up the other thread
    //     queueCondition.notify_one();

    //     // Sleep for a short duration (simulate Kinova feedback rate)
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
}

// Function for dynamic calculations
void performDynamicCalculations()
{
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

        // Perform dynamic calculations using the received feedback data
        // Print each element of the vectors
        std::cout << "Dynamic Calculations:" << std::endl;
        std::cout << "Received Position:" << std::endl;
        for (const auto &pos : data.positions)
        {
            std::cout << pos << std::endl;
        }

        std::cout << "Received Velocity:" << std::endl;
        for (const auto &vel : data.velocities)
        {
            std::cout << vel << std::endl;
        }

        std::cout << "Received Current:" << std::endl;
        for (const auto &curr : data.currents)
        {
            std::cout << curr << std::endl;
        }
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
    std::thread calculationsThread(performDynamicCalculations);

    // Join the threads to wait for them to finish (though they won't, as they are infinite loops)
    feedbackThread.join();
    calculationsThread.join();

    return 0;
}
