/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#ifndef QROBOTICSROBOTHAND_H
#define QROBOTICSROBOTHAND_H

#include <iostream>
#include <memory>
#include <qbrobotics_research_api/qbsofthand_research_api.h>
#include "robothand.h"


/**
 * @brief The RoboticHand interface
 *
 * Responsible for connection for its hand, and all other resources related to it.
 */
class QRoboticsRobotHand : public RobotHand
{
public:
    QRoboticsRobotHand( std::shared_ptr<qbrobotics_research_api::Communication> communicationHandler,
                        serial::PortInfo& serialPortInfo,
                        qbrobotics_research_api::Communication::ConnectedDeviceInfo& deviceInfo);

    virtual ~QRoboticsRobotHand();;


    // RobotHand interface
    const HandState getState() const {/* TODO */ return HandState{};}
    void setReference(const HandReference &hr) { /* TODO */ }
    const HandControlParameters getControlParameters() const {/* TODO */ return HandControlParameters{};};
    void setControlParameters(const HandControlParameters &hcp) {/* TODO */}

    // For printing with cout
    friend std::ostream& operator<<(std::ostream& os, /* const */ QRoboticsRobotHand& hand)
    {
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

#endif // QROBOTICSROBOTHAND_H
