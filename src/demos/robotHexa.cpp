#include <iostream>
#include "robotHexa.hpp"


void RobotHexa :: init()
{
    _controller.open_serial("/dev/ttyUSB0");

    // Scan actuators IDs
    _controller.scan_ax12s();
    const std::vector<byte_t>& ax12_ids = _controller.ax12_ids();
    if (!ax12_ids.size())
    {
        std::cerr<<"[ax12] no ax12 detected"<<std::endl;
        return;
    }
    std::cout << "[dynamixel] " << ax12_ids.size()
    << " dynamixel are connected" << std::endl;


    // Set wheels AX-12+ ids : [4, 8, 12, 16]
    _wheels_ids.push_back(13);
    _wheels_ids.push_back(14);
    _wheels_ids.push_back(15);
    _wheels_ids.push_back(16);
    _wheels_ids.push_back(17);
    _wheels_ids.push_back(18);

    // Set endless turn mode for wheels (p.17)
    for (size_t i = 0; i < _wheels_ids.size(); ++i)
    {
        _controller.send(dynamixel::ax12::SetContinuous(_wheels_ids[i]));
        _controller.recv(READ_DURATION, _status);
    }

    // Set AX-12+ ids : [1, 2, 3]
    _actuators_ids.push_back(1);
    _actuators_ids.push_back(31);
    _actuators_ids.push_back(21);
    _actuators_ids.push_back(7);

    _actuators_ids.push_back(2);
    _actuators_ids.push_back(32);
    _actuators_ids.push_back(22);
    _actuators_ids.push_back(8);

    _actuators_ids.push_back(3);
    _actuators_ids.push_back(33);
    _actuators_ids.push_back(23);
    _actuators_ids.push_back(9);

    _actuators_ids.push_back(4);
    _actuators_ids.push_back(34);
    _actuators_ids.push_back(24);
    _actuators_ids.push_back(10);

    _actuators_ids.push_back(5);
    _actuators_ids.push_back(35);
    _actuators_ids.push_back(25);
    _actuators_ids.push_back(11);

    _actuators_ids.push_back(6);
    _actuators_ids.push_back(36);
    _actuators_ids.push_back(26);
    _actuators_ids.push_back(12);

    std::cout << "initialisation completed" << std::endl;
}
void RobotHexa :: reset()
{
    std::cout << "setting all dynamixel to zero" << std::endl;
    enable();

    std::vector<int> pos(_actuators_ids.size());

    for (size_t i = 0; i < _actuators_ids.size(); ++i)
        if (_actuators_ids[i] >= 30) // mx28
            pos[i] = 1024;
        else
            pos[i] = 512;
    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);


    pos.clear();
    pos.resize(_actuators_ids.size());
    sleep(1);
    for (size_t i = 0; i < _actuators_ids.size(); ++i)
        switch ((int)_actuators_ids[i])
        {
        case 7:
            pos[i] = 700;
            break;
        case 8:
            pos[i] = 700;
            break;
        case 9:
            pos[i] = 1023;
            break;
        case 10:
            pos[i] = 00;
            break;
        case 11:
            pos[i] = 300;
            break;
        case 12:
            pos[i] = 300;
            break;

        default:
            if (_actuators_ids[i] >= 30) // mx28
                pos[i] = 1024;
            else
                pos[i] = 512;
            break;
        }


    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);

    pos.clear();
    pos.resize(_actuators_ids.size());
    sleep(1);
    for (size_t i = 0; i < _actuators_ids.size(); ++i)
        switch ((int)_actuators_ids[i])
        {
        case 7:
            pos[i] = 700;
            break;
        case 8:
            pos[i] = 700;
            break;
        case 9:
            pos[i] = 1023;
            break;
        case 10:
            pos[i] = 00;
            break;
        case 11:
            pos[i] = 300;
            break;
        case 12:
            pos[i] = 300;
            break;

        default:
            if (_actuators_ids[i] >= 30) // mx28
                pos[i] = 2048;
            else
                pos[i] = 512;
            break;
        }


    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);



    pos.clear();
    pos.resize(_actuators_ids.size());
    sleep(1);
    for (size_t i = 0; i < _actuators_ids.size(); ++i)
        if (_actuators_ids[i] == 8)
            pos[i] = 350;
        else if (_actuators_ids[i] == 11)
            pos[i] = 650;
        else if (_actuators_ids[i] >= 30) // mx28
            pos[i] = 2048;
        else
            pos[i] = 512;




    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);



    std::cout << "done" << std::endl;



}
void RobotHexa :: transfer(ControllerPhase& controller, float duration)
{

    float t=0;
    while (t<duration)
    {

      _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, controller.get_pos_dyna(t)));
      _controller.recv(READ_DURATION, _status);

      t+=0.01;
    }





}
