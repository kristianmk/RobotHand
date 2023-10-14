/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 *
 *  Note: Not completed, but the idea should be clear.
 */
#include <iostream>
#include <ostream>
#include <vector>
#include <optional>
#include <memory>

#include "qroboticsrobothand.h"
#include "serialportqroboticsrobothanddetector.h"


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


