#ifndef KINOVA_MOBILE_CONTROLLER_HPP
#define KINOVA_MOBILE_CONTROLLER_HPP

// #include <string>
#include <vector> // Include for std::vector
// #include <cstdint> // Include for uint8_t
// #include <CppLinuxSerial/SerialPort.hpp>

#include <string>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <cstdint>
#include <string>
#include <cstring>
#include <iostream>
#include <sys/socket.h> // Include for setsockopt

// Begin namespace declaration
namespace KinovaMobile
{
    // Forward declaration of KinovaMobileInterface class
    class KinovaMobileInterface;

    // Struct for Move reply
    struct MoveReply_t
    {
        float x;
        float y;
        float theta;
    };

    // Struct for Current Pose
    struct CurrentPose_t
    {
        float x;
        float y;
        float theta;
    };

    // Forward declaration of KinovaMobileProtocol class
    class KinovaMobileProtocol
    {
    public:
        // Enum for robot state
        enum class RobotState
        {
            Start = 0,
            Wait = 1,
            Move = 2,
            Reach = 3,
            Stop = 4,
            Reset = 5
        };

        // Struct for data exchange
        struct KinovaMobileData
        {
            int data;
            uint8_t reg;
            bool valid;
        };

        // Member functions
        static int EncodeFloat2Int(float value);
        static float DecodeInt2Float(int value);
        static void SendPacketData(KinovaMobileInterface *myInterface, const KinovaMobileData &data);
        static std::vector<uint8_t> ReceiveDataUntilHeader(KinovaMobileInterface *myInterface);
        static KinovaMobileData ProcessingReceivedData(const std::vector<uint8_t> &buffer);

        // Constants
        static constexpr uint8_t HEAD_BYTE = 0x1D;
        static constexpr uint8_t ESCAPE_BYTE = 0x1E;
        static constexpr uint8_t ESCAPE_MASK = 0x1F;
        static constexpr uint8_t COMMAND_MOVE = 0x04;
        static constexpr uint8_t COMMAND_POSE_X = 0x05;
        static constexpr uint8_t COMMAND_POSE_Y = 0x06;
        static constexpr uint8_t COMMAND_POSE_THETA = 0x07;

        static constexpr uint8_t COMMAND_VELOCITY_X = 0x2A;
        static constexpr uint8_t COMMAND_VELOCITY_Y = 0x3A;
        static constexpr uint8_t COMMAND_VELOCITY_THETA = 0x4A;

        static constexpr uint8_t COMMAND_RESET_ODOMETRY = 0x015;

        static constexpr uint8_t COMMAND_STOP = 0xFF;

        static constexpr uint8_t REPLY_ROBOT_STATE = 0x03;
        static constexpr uint8_t REPLY_MOVE = 0x04;
        static constexpr uint8_t REPLY_COMMAND_X = 0x05;
        static constexpr uint8_t REPLY_COMMAND_Y = 0x06;
        static constexpr uint8_t REPLY_COMMAND_THETA = 0x07;
        static constexpr uint8_t REPLY_STATE_X = 0x011;
        static constexpr uint8_t REPLY_STATE_Y = 0x012;
        static constexpr uint8_t REPLY_STATE_THETA = 0x13;
        static constexpr uint8_t REPLY_RESET_ODOMETRY = 0x15;

        static constexpr uint8_t DEBUG_CONSOLE = 0x20;
    };

    // Forward declaration of KinovaMobileInterface class
    class KinovaMobileInterface
    {
    public:
        virtual ~KinovaMobileInterface() {}

        virtual void DiscardInBuffer() = 0;
        virtual void Close() = 0;
        virtual void SendByteData(uint8_t data) = 0;
        virtual void SendBytes(const uint8_t *buffer, int size) = 0;
        virtual uint8_t ReadOneByteData() = 0;
        virtual int ReadBytes(uint8_t *buffer, int size) = 0;
    };

    // Forward declaration of KinovaMobileSerial class
    class KinovaMobileSerial;

    // KinovaMobileController class declaration
    class KinovaMobileController
    {
    public:
        // Constructor
        KinovaMobileController(const std::string &portOrDeviceName);

        // Destructor
        ~KinovaMobileController();

        // Member functions
        void ReceiveData();
        void SendRefPose(float x, float y, float theta);
        void SendRefVelocities(float x, float y, float theta);

        bool IsCommandPoseCorrect(float x, float y, float theta);
        void Move();
        void ResetOdometry();
        bool IsMoving();
        bool IsReached();
        bool IsStopping();
        void Stop();
        void CloseInterface();

        // Member Var
        // int* registerArray;

    private:
        // Private members
        KinovaMobileInterface *myInterface;

        // static constexpr int arraySize = 0x30;
        // int register[arraySize];

        // std::vector<int> register(0x30);
        // unsigned char register[] {0x30};
        // std::vector<int> register;

        int registerArray[0x30];

        MoveReply_t moveCommandReply;
        CurrentPose_t currentPoseReply;
        KinovaMobileProtocol::RobotState robotStateReply;
        bool stopCommandReply;
        bool isReceived = false;
    };

    // KinovaMobileSerial class declaration
    class KinovaMobileSerial : public KinovaMobileInterface
    {
    public:
        // Constructor
        KinovaMobileSerial(const std::string &port_name);

        // Destructor
        ~KinovaMobileSerial();

        // Overridden member functions
        void Close() override;
        void DiscardInBuffer() override;
        uint8_t ReadOneByteData() override;
        int ReadBytes(uint8_t *buffer, int size);
        void SendByteData(uint8_t data) override;
        void SendBytes(const uint8_t *buffer, int size) override;

    private:
        // mn::CppLinuxSerial::SerialPort* port;
        int port; // File descriptor for the serial port

        void WriteOneByteData(uint8_t data);
    };

    // End namespace declaration
}

#endif // KINOVA_MOBILE_CONTROLLER_HPP
