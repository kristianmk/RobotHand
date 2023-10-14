/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#include "qroboticsrobothand.h"


QRoboticsRobotHand::QRoboticsRobotHand(std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler, serial::PortInfo &serialPortInfo, qbrobotics_research_api::Communication::ConnectedDeviceInfo &deviceInfo)
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

QRoboticsRobotHand::~QRoboticsRobotHand()
{
    // Here we should probably close the port only if this is the last hand using this serial port. Now it will close it
    // without caring about other users.
    const bool successClosing = communicationHandler_->closeSerialPort(serialPortInfo_.serial_port) == 0;

    if(successClosing)
    {
        std::cout << "Serial port " << serialPortInfo_.serial_port << " successfully closed." <<  std::endl;
    }
}

