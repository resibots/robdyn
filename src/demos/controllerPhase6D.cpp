#include "controllerPhase6D.hpp"

#define RAD2DYN 195.57
void ControllerPhase::moveRobot(robot_t& robot, float t)
{

  //std::cout<<"debut move"<<std::endl;
    size_t leg = 0;
    for (size_t i = 0; i < robot->servos().size(); i+=3)
    {
      //    std::cout<<"dans move"<<std::endl;
       for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg==_brokenLegs[j])
            {
                leg++;
                if (_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=leg)
                    break;
            }
	    }

        robot->servos()[i]->set_angle(0,_legsParams[leg][0]*M_PI/8+ _legsParams[leg][1]*M_PI/8*delayedPhase(t,_legsParams[leg][2]));

	/*if (!isBroken(leg))
	  {*/
	    robot->servos()[i + 1]->set_angle(0,_legsParams[leg][3]*M_PI/4+_legsParams[leg][4]*M_PI/4*delayedPhase(t,_legsParams[leg][5]));
	    /* }
	else
	  {
	    robot->servos()[i + 1]->set_angle(0,-M_PI/4);
	    }*/
	robot->servos()[i + 2]->set_angle(0,-_legsParams[leg][3]*M_PI/4-_legsParams[leg][4]*M_PI/4*delayedPhase(t,_legsParams[leg][6]));
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
        //servo 0
        float theta0=_legsParams[leg][0]*M_PI/8+ _legsParams[leg][1]*M_PI/8*delayedPhase(t,_legsParams[leg][2]);
        //theta0=theta0>M_PI/8?M_PI/8:theta0;
        //theta0=theta0<-M_PI/8?-M_PI/8:theta0;
	if(leg==0 ||leg ==3)
	  pos.push_back(512-64-RAD2DYN*(theta0));
	else if(leg == 2 || leg==5)
	  pos.push_back(512+64-RAD2DYN*(theta0));
	else
	  pos.push_back(512-RAD2DYN*(theta0));

        //servo 1
        //setting an offset for mx28s which have bad zero
        float theta1=_legsParams[leg][3]*M_PI/4+_legsParams[leg][4]*M_PI/4*delayedPhase(t,_legsParams[leg][5]);
        //theta1=theta1>M_PI/4?M_PI/4:theta1;
        //theta1=theta1<-M_PI/4?-M_PI/4:theta1;
        if (leg==0)
            pos.push_back(2048+50+RAD2DYN*4*(theta1));
        else if (leg==3)
            pos.push_back(2048-150+RAD2DYN*4*(theta1));
        else if (leg == 4)
            pos.push_back(2048-200+RAD2DYN*4*(theta1));
        else
            pos.push_back(2048+RAD2DYN*4*(theta1));


        //servo 2
        float theta2=-_legsParams[leg][3]*M_PI/4-_legsParams[leg][4]*M_PI/4*delayedPhase(t,_legsParams[leg][6]);
        //theta2=theta0>M_PI/4?M_PI/4:theta2;
        //theta2=theta0<-M_PI/4?-M_PI/4:theta2;
        pos.push_back(512-RAD2DYN*(theta2));
        //std::cout<<"leg "<<leg <<" pos theta2:" << 512-RAD2DYN*(theta2)<<std::endl;
        ++leg;
    }
    return pos;
}




float ControllerPhase::delayedPhase(float t, float phi)
{

    return tanh(sin(2*M_PI*t+phi*2*M_PI)*4);

}

