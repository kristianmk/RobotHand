/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 *
 *  Note: Not completed, but the idea should be clear. Should be refactored before further work.
 */
#include <iostream>
#include <ostream>
#include <vector>
#include <optional>
#include <memory>
#include <qbrobotics_research_api/qbsofthand_research_api.h>


class RobotHand;
class QRoboticsRobotHand;
class SerialPortQRoboticsRobotHandDetector;


/**
 * @brief The HandDetector interface
 *
 * Factory interface class that will produce a RoboticHand for each detected available qbsofthand.
 */
class QRoboticsRobotHandDetector
{
public:
    virtual ~QRoboticsRobotHandDetector() {};
    virtual unsigned int detectHands() = 0;
    virtual std::vector<std::unique_ptr<RobotHand> > getDetectedHands() = 0;
};


/**
 * @brief The SerialPortHandDetector class
 *
 * Factory class that will produce a RoboticHand for each detected available qbsofthand.
 */
class SerialPortQRoboticsRobotHandDetector : public QRoboticsRobotHandDetector
{
public:
    SerialPortQRoboticsRobotHandDetector()
        : serialPorts_{}
        , communicationHandler_{std::make_shared<qbrobotics_research_api::CommunicationLegacy>()}
        , detectedRoboticHands_{}
    {
        // Do nothing.
    }

    unsigned int detectHands()
    {
        // Find all serial ports with a qbrobotics device connected.
        const int numberOfPortsWithDetectedDevice = communicationHandler_->listSerialPorts(serialPorts_);
        std::cout << "Detected " << numberOfPortsWithDetectedDevice << " ports with qrobotics devices." << std::endl;

        if (numberOfPortsWithDetectedDevice <= 0)
        {
            return 0U;
        }

        // Open all ports with a detected device.
        for (auto& serialPort : serialPorts_)
        {
            const bool openSuccessful = communicationHandler_->openSerialPort(serialPort.serial_port) >= 0 ? true : false;

            if (!openSuccessful)
            {
                std::cout << "Could not open serial port " << serialPort.serial_port << "." << std::endl;
                continue;
            }

            std::cout << "Serial port " << serialPort.serial_port << " opened successfully." << std::endl;


            std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> devicesInfo;
            const bool listSuccessful = communicationHandler_->listConnectedDevices(serialPort.serial_port, devicesInfo) >= 0 ? true : false;

            if (!listSuccessful)
            {
                std::cout << "Could not list devices." << std::endl;
                continue;
            }

            for (auto& deviceInfo : devicesInfo)
            {
                if (deviceInfo.id == 0)
                {
                    std::cout << "Invalid device ID=0" << std::endl;
                    continue;
                }

                if (deviceInfo.id == 120)
                {
                    std::cout << "Invalid device ID=120 (reserved!)" << std::endl;
                    continue;
                }

                std::cout << "Found valid device ID=" << deviceInfo.id << std::endl;
                detectedRoboticHands_.emplace_back(std::make_unique<QRoboticsRobotHand>(communicationHandler_, serialPort, deviceInfo));
            }
        }

        return detectedRoboticHands_.size();
    }

    std::vector<std::unique_ptr<RobotHand> > getDetectedHands()
    {
        return std::move(detectedRoboticHands_); // detectedRoboticHands_ are now nullptr.
    }

private:
    std::vector<serial::PortInfo> serialPorts_;
    std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler_;
    std::vector<std::unique_ptr<RobotHand> > detectedRoboticHands_;
};


struct HandState
{
    // Important part: optional!
    std::optional<double> current; // Just an example.
    std::optional<double> position; // Same. Note: double is not the correct data type here.
};


struct HandReference
{
    // Important part: optional! How to structure this message depends on number of joints and topology.
    std::optional<double> position; // Just an example. Probably need this in 3D. And for many joints.
    std::optional<double> velocity; // Just an example. Probably need this in 3D. And for many joints.
    std::optional<double> acceleration; // Just an example. Probably need this in 3D. And for many joints.
};

struct HandControlParameters
{
    // Something here (PID parameters and so on...).
};


/**
 * @brief The RobotHand interface
 *
 * Responsible for connection for its hand, and all other resources related to it.
 */
class RobotHand
{
public:
    virtual ~RobotHand() {};
    virtual const HandState getState() const = 0;
    virtual void setReference(const HandReference& hr) = 0;
    virtual const HandControlParameters getControlParameters() const = 0;
    virtual void setControlParameters(const HandControlParameters& hcp) = 0;
};


/**
 * @brief The RoboticHand interface
 *
 * Responsible for connection for its hand, and all other resources related to it.
 */
class QRoboticsRobotHand : public RobotHand
{
public:
    QRoboticsRobotHand(std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler, serial::PortInfo& serialPortInfo, qbrobotics_research_api::Communication::ConnectedDeviceInfo& deviceInfo)
        : serialPortInfo_{serialPortInfo}
        , communicationHandler_{communicationHandler}
        , deviceInfo_{deviceInfo}
        , qbSoftHandLegacyResearch_{communicationHandler,
                                    "dev",
                                    serialPortInfo_.serial_port,
                                    deviceInfo_.id}
    {
        // Do nothing.
        std::cout << "Creating RobotHand.." << std::endl;
    }

    virtual ~QRoboticsRobotHand()
    {
        const bool successClosing = communicationHandler_->closeSerialPort(serialPortInfo_.serial_port) == 0;

        if(successClosing)
        {
          std::cout << "Serial port " << serialPortInfo_.serial_port << " successfully closed." <<  std::endl;
        }
    };


    // RobotHand interface
    const HandState getState() const {/* TODO */ return HandState{};}
    void setReference(const HandReference &hr) { /* TODO */ }
    const HandControlParameters getControlParameters() const {/* TODO */ return HandControlParameters{};};
    void setControlParameters(const HandControlParameters &hcp) {/* TODO */}

    // For printing with cout
    friend std::ostream& operator<<(std::ostream& os, /* const */ QRoboticsRobotHand& hand) {
        std::string info;
        hand.qbSoftHandLegacyResearch_.getInfo(INFO_ALL, info);
        os << info;
        return os;
    }


private:
    serial::PortInfo serialPortInfo_;
    std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler_;
    qbrobotics_research_api::Communication::ConnectedDeviceInfo deviceInfo_;
    qbrobotics_research_api::qbSoftHandLegacyResearch qbSoftHandLegacyResearch_;
};



int main()
{
    std::cout << "Starting up HandTest.." << std::endl;


    std::cout << "Creating HandDetector.." << std::endl;
    SerialPortQRoboticsRobotHandDetector* handDetector = new SerialPortQRoboticsRobotHandDetector();

    std::cout << "Detecting hands.." << std::endl;
    const unsigned int numberOfDevicesFound = handDetector->detectHands();

    std::cout << "Getting detected hands.." << std::endl;
    auto robotHands = handDetector->getDetectedHands();


    delete handDetector; // TODO: Important. Check how the library releases resources. Then switch back to automatic variable.


    std::cout << "Located " << numberOfDevicesFound << " qbrobotics devices." << std::endl;
    std::cout << "Located " << robotHands.size() << " qbrobotics devices." << std::endl;



    std::cout << "Shutting down HandTest.." << std::endl;

    return 0;
}


