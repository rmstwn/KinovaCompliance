#include "KinovaMobile/kinova_mobile.hpp"

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
#include <sys/ioctl.h>

namespace KinovaMobile
{

    // KinovaMobileProtocol implementation
    int KinovaMobileProtocol::EncodeFloat2Int(float value)
    {
        // Implementation of encoding float to int
        // Example implementation (replace with your encoding logic)
        return static_cast<int>(value * 1000);
    }

    float KinovaMobileProtocol::DecodeInt2Float(int value)
    {
        // Implementation of decoding int to float
        // Example implementation (replace with your decoding logic)
        return static_cast<float>(value) / 1000.0;
    }

    void KinovaMobileProtocol::SendPacketData(KinovaMobileInterface *myInterface, const KinovaMobileData &data_t)
    {
        uint8_t dataBytes[4] = {
            static_cast<uint8_t>((data_t.data >> 24) & 0xFF),
            static_cast<uint8_t>((data_t.data >> 16) & 0xFF),
            static_cast<uint8_t>((data_t.data >> 8) & 0xFF),
            static_cast<uint8_t>((data_t.data >> 0) & 0xFF)};

        std::vector<uint8_t> buffer_w;

        buffer_w.push_back(HEAD_BYTE);
        buffer_w.push_back(data_t.reg);

        uint8_t checksum = data_t.reg;
        for (int i = 0; i < 4; ++i)
        {
            if ((dataBytes[i] == ESCAPE_BYTE) || (dataBytes[i] == HEAD_BYTE))
            {
                buffer_w.push_back(ESCAPE_BYTE);
                checksum += ESCAPE_BYTE;
                buffer_w.push_back(static_cast<uint8_t>(dataBytes[i] ^ ESCAPE_MASK));
                checksum += static_cast<uint8_t>(dataBytes[i] ^ ESCAPE_MASK);
            }
            else
            {
                buffer_w.push_back(dataBytes[i]);
                checksum += dataBytes[i];
            }
        }

        // Add checksum to the end of the packet
        buffer_w.push_back(checksum);
        int size = buffer_w.size() - 1;
        buffer_w.insert(buffer_w.begin() + 1, static_cast<uint8_t>(size));

        // Convert vector to array
        std::vector<uint8_t> buffer(buffer_w.begin(), buffer_w.end());

        // Send the packet through the interface
        myInterface->SendBytes(buffer.data(), buffer.size());
    }

    std::vector<uint8_t> KinovaMobileProtocol::ReceiveDataUntilHeader(KinovaMobileInterface *myInterface)
    {
        std::vector<uint8_t> buffer;

        uint8_t indata = myInterface->ReadOneByteData();
        // Ignore data until HEAD_BYTE is received
        while (indata != HEAD_BYTE)
        {
            indata = myInterface->ReadOneByteData();
        }

        // Read the size of the packet
        int size = myInterface->ReadOneByteData();
        buffer.push_back(HEAD_BYTE);
        buffer.push_back(static_cast<uint8_t>(size));

        // Read the packet data
        for (int i = 0; i < size; ++i)
        {
            buffer.push_back(myInterface->ReadOneByteData());
        }

        return buffer;
    }

    KinovaMobileProtocol::KinovaMobileData KinovaMobileProtocol::ProcessingReceivedData(const std::vector<uint8_t> &buffer)
    {
        KinovaMobileData data_t;

        if (buffer.empty())
        {
            data_t.data = 0;
            data_t.reg = 0;
            data_t.valid = false;
            return data_t;
        }

        size_t index = 0;
        uint8_t reg = buffer[index++];
        uint8_t checksum = reg;

        std::vector<uint8_t> bytes(4, 0);

        for (int i = 0; i < 4; ++i)
        {
            uint8_t d = buffer[index++];
            if (d == ESCAPE_BYTE)
            {
                uint8_t nextByte = buffer[index++];
                bytes[i] = nextByte ^ ESCAPE_MASK;
                checksum += d + nextByte;
            }
            else
            {
                bytes[i] = d;
                checksum += d;
            }
        }

        uint8_t checksum_recv = buffer[index++];
        uint32_t DATA = 0x00;
        for (int i = 0; i < 4; i++)
        {
            DATA |= (static_cast<uint32_t>(bytes[i]) << (24 - (i * 8)));
        }

        if (checksum == checksum_recv)
        {
            data_t.data = DATA;
            data_t.reg = reg;
            data_t.valid = true;
        }
        else
        {
            // Data error
            // std::cout << "Data error, checksum is wrong." << std::endl;
            data_t.data = 0;
            data_t.reg = 0;
            data_t.valid = false;
        }

        return data_t;
    }

    // KinovaMobileController implementation
    KinovaMobileController::KinovaMobileController(const std::string &portOrDeviceName)
    {
        myInterface = new KinovaMobileSerial(portOrDeviceName);
    }

    KinovaMobileController::~KinovaMobileController()
    {
        delete myInterface;
    }

    void KinovaMobileController::ReceiveData()
    {
        // Receive data until header
        std::vector<uint8_t> buffer = KinovaMobileProtocol::ReceiveDataUntilHeader(myInterface);

        // Process received data
        KinovaMobileProtocol::KinovaMobileData data_t = KinovaMobileProtocol::ProcessingReceivedData(buffer);

        // Check if received data is valid
        if (data_t.valid)
        {
            isReceived = true;
            registerArray[data_t.reg] = data_t.data;

            // Switch based on the received register
            switch (data_t.reg)
            {
            case KinovaMobileProtocol::REPLY_ROBOT_STATE:
                robotStateReply = static_cast<KinovaMobileProtocol::RobotState>(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_MOVE:
                // Handle reply move
                break;
            case KinovaMobileProtocol::REPLY_COMMAND_X:
                moveCommandReply.x = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_COMMAND_Y:
                moveCommandReply.y = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_COMMAND_THETA:
                moveCommandReply.theta = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_STATE_X:
                currentPoseReply.x = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_STATE_Y:
                currentPoseReply.y = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            case KinovaMobileProtocol::REPLY_STATE_THETA:
                currentPoseReply.theta = KinovaMobileProtocol::DecodeInt2Float(data_t.data);
                break;
            default:
                // Handle unknown register
                break;
            }
        }
    }

    void KinovaMobileController::SendRefPose(float x, float y, float theta)
    {
        // Encode float values into integers
        int xEncoded = KinovaMobileProtocol::EncodeFloat2Int(x);
        int yEncoded = KinovaMobileProtocol::EncodeFloat2Int(y);
        int thetaEncoded = KinovaMobileProtocol::EncodeFloat2Int(theta);

        // Create packet data
        KinovaMobileProtocol::KinovaMobileData dataPacket;

        // // Send x-coordinate command
        // dataPacket.data = xEncoded;
        // dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_X;
        // dataPacket.valid = true;
        // KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);

        // // Send y-coordinate command
        // dataPacket.data = yEncoded;
        // dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_Y;
        // KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);

        // // Send theta command
        // dataPacket.data = thetaEncoded;
        // dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_THETA;
        // KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);

        // Send x-coordinate command
        dataPacket.data = xEncoded;
        dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_X;
        dataPacket.valid = true;
        std::cout << "X-coordinate packet: data=" << dataPacket.data << ", reg=" << static_cast<int>(dataPacket.reg) << ", valid=" << dataPacket.valid << std::endl;
        KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);

        // Send y-coordinate command
        dataPacket.data = yEncoded;
        dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_Y;
        std::cout << "Y-coordinate packet: data=" << dataPacket.data << ", reg=" << static_cast<int>(dataPacket.reg) << ", valid=" << dataPacket.valid << std::endl;
        KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);

        // Send theta command
        dataPacket.data = thetaEncoded;
        dataPacket.reg = KinovaMobileProtocol::COMMAND_POSE_THETA;
        std::cout << "Theta packet: data=" << dataPacket.data << ", reg=" << static_cast<int>(dataPacket.reg) << ", valid=" << dataPacket.valid << std::endl;
        KinovaMobileProtocol::SendPacketData(myInterface, dataPacket);
    }

    bool KinovaMobileController::IsCommandPoseCorrect(float x, float y, float theta)
    {
        bool correct = true;
        correct &= std::abs(moveCommandReply.x - x) < 0.001f;
        correct &= std::abs(moveCommandReply.y - y) < 0.001f;
        correct &= std::abs(moveCommandReply.theta - theta) < 0.001f;
        return correct;
    }

    void KinovaMobileController::Move()
    {
        KinovaMobileProtocol::KinovaMobileData data_t;
        data_t.data = 1;
        data_t.reg = KinovaMobileProtocol::COMMAND_MOVE;
        data_t.valid = true;
        KinovaMobileProtocol::SendPacketData(myInterface, data_t);
    }

    void KinovaMobileController::ResetOdometry()
    {
        KinovaMobileProtocol::KinovaMobileData data_t;
        data_t.data = 1;
        data_t.reg = KinovaMobileProtocol::COMMAND_RESET_ODOMETRY;
        data_t.valid = true;
        KinovaMobileProtocol::SendPacketData(myInterface, data_t);
    }

    bool KinovaMobileController::IsMoving()
    {
        return (robotStateReply == KinovaMobileProtocol::RobotState::Move);
    }

    bool KinovaMobileController::IsReached()
    {
        return (robotStateReply == KinovaMobileProtocol::RobotState::Reach);
    }

    bool KinovaMobileController::IsStopping()
    {
        return (robotStateReply == KinovaMobileProtocol::RobotState::Stop);
    }

    void KinovaMobileController::Stop()
    {
        KinovaMobileProtocol::KinovaMobileData data_t;
        data_t.data = 0;
        data_t.reg = KinovaMobileProtocol::COMMAND_MOVE;
        data_t.valid = true;
        KinovaMobileProtocol::SendPacketData(myInterface, data_t);
    }

    void KinovaMobileController::CloseInterface()
    {
        myInterface->Close();
    }

    // KinovaMobileSerial implementation
    KinovaMobileSerial::KinovaMobileSerial(const std::string &port_name)
    {
        // // port = new SerialPort(port_name, baud_rate, Parity::None, 8, StopBits::One);
        // // port = new mn::CppLinuxSerial::SerialPort(port_name, mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE);
        // // port->ReadBufferSize = 64;
        // // mn::CppLinuxSerial::SerialPort port(port_name, mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE);

        // try
        // {
        //     port.SetTimeout(100); // Block for up to 100ms to receive data
        //     port.Open();
        //     // port->Open();
        //     // port->DtrEnable = true;
        //     // port->RtsEnable = true;
        //     std::cout << "Connected." << std::endl;
        // }
        // catch (const std::exception &err)
        // {
        //     std::cout << "Unexpected exception: " << err.what() << std::endl;
        // }

        // Open the serial port
        port = open(port_name.c_str(), O_RDWR | O_NOCTTY);
        if (port == -1)
        {
            std::cerr << "Failed to open serial port." << std::endl;
            // Handle error
            return;
        }

        // Configure serial port settings
        struct termios options;
        tcgetattr(port, &options);
        cfsetispeed(&options, B115200); // Set baud rate
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
        options.c_cflag &= ~PARENB;          // Disable parity
        options.c_cflag &= ~CSTOPB;          // Set one stop bit
        options.c_cflag &= ~CSIZE;           // Mask the character size bits
        options.c_cflag |= CS8;              // Set 8 data bits
        tcsetattr(port, TCSANOW, &options);

        // Enable DTR signal
        int status;
        if (ioctl(port, TIOCMGET, &status) == -1)
        {
            std::cerr << "Failed to get serial port status." << std::endl;
            // Handle error or set internal state to indicate error
        }
        else
        {
            status |= TIOCM_DTR;
            if (ioctl(port, TIOCMSET, &status) == -1)
            {
                std::cerr << "Failed to set DTR signal." << std::endl;
                // Handle error or set internal state to indicate error
            }
        }

        // Enable RTS signal
        if (ioctl(port, TIOCMGET, &status) == -1)
        {
            std::cerr << "Failed to get serial port status." << std::endl;
            // Handle error or set internal state to indicate error
        }
        else
        {
            status |= TIOCM_RTS;
            if (ioctl(port, TIOCMSET, &status) == -1)
            {
                std::cerr << "Failed to set RTS signal." << std::endl;
                // Handle error or set internal state to indicate error
            }
        }

        // Set timeout (optional)
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms timeout
        setsockopt(port, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

        std::cout << "Connected." << std::endl;
    }

    KinovaMobileSerial::~KinovaMobileSerial()
    {
        Close(); // Call Close function in the destructor
    }

    void KinovaMobileSerial::Close()
    {
        if (port != -1)
        {
            close(port);
            port = -1; // Reset port to indicate it's closed
        }
    }

    void KinovaMobileSerial::DiscardInBuffer()
    {
        // Flushes data received but not read from the serial port input buffer
        if (tcflush(port, TCIFLUSH) != 0)
        {
            std::cerr << "Error flushing input buffer of serial port: " << strerror(errno) << std::endl;
        }
    }

    uint8_t KinovaMobileSerial::ReadOneByteData()
    {

        try
        {
            uint8_t data;                                        // Variable to store the read byte
            ssize_t bytesRead = read(port, &data, sizeof(data)); // Read one byte of data
            if (bytesRead < 0)
            {
                // Error handling if read fails
                std::cerr << "Error reading from serial port." << std::endl;
                return 0; // Return 0 on error
            }
            else if (bytesRead == 0)
            {
                // No data available
                return 0;
            }
            else
            {
                return data; // Return the read byte
            }
        }
        catch (const std::exception &)
        {
            return 0;
        }
    }

    int KinovaMobileSerial::ReadBytes(uint8_t *buffer, int size)
    {
        try
        {
            // Read data from the serial port into the provided buffer
            ssize_t bytesRead = read(port, buffer, size);
            if (bytesRead < 0)
            {
                // Error handling if read fails
                std::cerr << "Error reading from serial port." << std::endl;
                return -1; // Return -1 on error
            }
            return bytesRead; // Return the number of bytes read
        }
        catch (const std::exception &)
        {
            return -1;
        }
    }

    void KinovaMobileSerial::WriteOneByteData(uint8_t data)
    {
        try
        {
            uint8_t buffer[1] = {data};
            ssize_t bytesWritten = write(port, buffer, sizeof(buffer));
            if (bytesWritten == -1)
            {
                std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Exception occurred while writing to serial port." << std::endl;
        }
    }

    void KinovaMobileSerial::SendByteData(uint8_t data)
    {
        WriteOneByteData(data);
    }

    void KinovaMobileSerial::SendBytes(const uint8_t *buffer, int size)
    {
        try
        {
            ssize_t bytesWritten = write(port, buffer, size);
            if (bytesWritten == -1)
            {
                std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Exception occurred while writing to serial port." << std::endl;
        }
    }

} // End namespace KinovaMobile
