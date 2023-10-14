#ifndef SERIALPORTQROBOTICSROBOTHANDDETECTOR_H
#define SERIALPORTQROBOTICSROBOTHANDDETECTOR_H

/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#include "qroboticsrobothanddetector.h"
#include <qbrobotics_research_api/qbsofthand_research_api.h>


/**
 * @brief The SerialPortHandDetector class
 *
 * Factory class that will produce a RoboticHand for each detected available qbsofthand.
 */
class SerialPortQRoboticsRobotHandDetector : public QRoboticsRobotHandDetector
{
public:
    SerialPortQRoboticsRobotHandDetector();

    unsigned int detectHands();

    inline std::vector<std::unique_ptr<RobotHand> > getDetectedHands()
    {
        return std::move(detectedRoboticHands_); // detectedRoboticHands_ is now nullptr.
    }

private:
    std::vector<serial::PortInfo> serialPorts_;
    std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler_;
    std::vector<std::unique_ptr<RobotHand> > detectedRoboticHands_;
};
#endif // SERIALPORTQROBOTICSROBOTHANDDETECTOR_H
