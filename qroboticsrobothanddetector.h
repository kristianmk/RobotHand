/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#ifndef QROBOTICSROBOTHANDDETECTOR_H
#define QROBOTICSROBOTHANDDETECTOR_H

#include <memory>
#include <vector>
#include "robothand.h"

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

#endif // QROBOTICSROBOTHANDDETECTOR_H
