/**
 *  Written by K. M. Knausgård 2023-10-13. All rights reserved.
 */
#ifndef ROBOTHAND_H
#define ROBOTHAND_H

#include <optional>

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

#endif // ROBOTHAND_H
