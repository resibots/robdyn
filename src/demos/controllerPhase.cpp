#include "controllerPhase.hpp"

#define RAD2DYN 195.57
void ControllerPhase::moveRobot(robot_t& robot, float t)
{

    //std::cout<<"debut move"<<std::endl;
    size_t leg = 0;
    for (size_t i = 0; i < robot->servos().size(); i+=3)
    {
        //std::cout<<"dans move"<<std::endl;
        for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg==_brokenLegs[j])
            {
                leg++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg)
                    break;
            }
        }

        robot->servos()[i]->set_angle(0,_legsParams[leg][0]*M_PI/3+ _legsParams[leg][1]*M_PI/3*delayedPhase(t,_legsParams[leg][2]));
        robot->servos()[i + 1]->set_angle(0,_legsParams[leg][3]*M_PI/3+_legsParams[leg][4]*delayedPhase(t,_legsParams[leg][5]));
        robot->servos()[i + 2]->set_angle(0,_legsParams[leg][3]*M_PI/3-_legsParams[leg][4]*delayedPhase(t,_legsParams[leg][6]));

	// robot->servos()[i + 3]->set_angle(0,0);
        ++leg;
    }

}



std::vector<int> ControllerPhase::get_pos_dyna(float t)
{
    std::vector<int> pos;
    //std::cout<<"debut move"<<std::endl;
    size_t leg = 0;
    for (size_t i = 0; i < 24; i+=4)
    {
        //std::cout<<"dans move"<<std::endl;
        for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg==_brokenLegs[j])
            {
                leg++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg)
                    break;
            }
        }

        pos.push_back(512*RAD2DYN*(_legsParams[leg][0]*M_PI/3+ _legsParams[leg][1]*M_PI/3*delayedPhase(t,_legsParams[leg][2])));
        pos.push_back(512*RAD2DYN*(_legsParams[leg][3]*M_PI/3+_legsParams[leg][4]*delayedPhase(t,_legsParams[leg][5])));
        pos.push_back(512*RAD2DYN*(_legsParams[leg][3]*M_PI/3+_legsParams[leg][4]*delayedPhase(t,_legsParams[leg][6])));

        pos.push_back(512);
        ++leg;
    }
    std::cout<<"taille pos:"<<pos.size()<<std::endl;
  return pos;
}




float ControllerPhase::delayedPhase(float t, float phi)
{

    return tanh(sin(2*M_PI*t+phi*2*M_PI)*4);

}
