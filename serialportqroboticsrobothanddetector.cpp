/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#include "serialportqroboticsrobothanddetector.h"

#include <iostream>
#include <qbrobotics_research_api/qbsofthand_research_api.h>

#include "qroboticsrobothand.h"


SerialPortQRoboticsRobotHandDetector::SerialPortQRoboticsRobotHandDetector()
    : serialPorts_{}
    , communicationHandler_{std::make_shared<qbrobotics_research_api::CommunicationLegacy>()}
    , detectedRoboticHands_{}
{
    // Do nothing.
}

unsigned int SerialPortQRoboticsRobotHandDetector::detectHands()
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
