#ifndef ROBOTHEXA_HPP
#define ROBOTHEXA_HPP

#include <stdlib.h>
#include <math.h>
#include <dynamixel/dynamixel.hpp>
#include "controllerPhase.hpp"

#define DEFAULT_SPEED 150
#define READ_DURATION 0.02f
#define SAMPLING_FREQUENCY 20

class RobotHexa
{
public:
    typedef unsigned char byte_t;

    RobotHexa()
    {
        init();
    }
    void init();

    void relax()
    {
        std::cout << "relax..." << std::endl;
        for (size_t i = 0; i < _actuators_ids.size(); ++i)
        {
            _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], false));
            _controller.recv(READ_DURATION, _status);
        }
        std::cout << "done" << std::endl;
    }
    void enable()
    {
        for (size_t i = 0; i < _actuators_ids.size(); ++i)
        {
            _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], true));
            _controller.recv(READ_DURATION, _status);
        }
        usleep(1e5);
    }
    void reset();
    void transfer(ControllerPhase& controller, float duration);
    size_t nb_actuators() const
    {
        return _actuators_ids.size();
    }
    size_t nb_wheels() const
    {
        return _wheels_ids.size();
    }
protected:

    dynamixel::Usb2Dynamixel _controller;
    dynamixel::Status _status;
    std::vector<byte_t> _wheels_ids;

    std::vector<byte_t> _actuators_ids;

};

#endif
