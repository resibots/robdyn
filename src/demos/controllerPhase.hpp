#ifndef CONTROLLERPHASE_HPP
#define CONTROLLERPAHSE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include "robot/hexapod.hh"





class ControllerPhase
{

protected :

    std::vector< std::vector<float> > _legsParams;
    std::vector<int> _brokenLegs;
private:
    float delayedPhase(float t, float phi);

public :
    typedef boost::shared_ptr<robot::Robot> robot_t;
    ControllerPhase(const std::vector<float>& ctrl,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
    {


        assert(ctrl.size()==42);
        for(int leg=0;leg<6;leg++)
        {
            std::vector<float> param;
            param.push_back(ctrl[leg*7]*2-1);
            param.push_back(ctrl[leg*7+1]);
            param.push_back(ctrl[leg*7+2]);
            param.push_back(ctrl[leg*7+3]*2-1);
            param.push_back(ctrl[leg*7+4]);
            param.push_back(ctrl[leg*7+5]);
            param.push_back(ctrl[leg*7+6]);
            _legsParams.push_back(param);
        }

    }

    void moveRobot(robot_t& robot, float t);
    std::vector<int> get_pos_dyna(float t);



};

#endif
