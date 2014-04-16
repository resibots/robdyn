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
  typedef boost::shared_ptr<robot::Hexapod> robot_t;



  bool isBroken(int leg)
  {
    for (int j=0;j<_brokenLegs.size();j++)
      {
	if (leg==_brokenLegs[j])
	  {
	    return true;
	  }
      }
    return false;
  }


  ControllerPhase(const std::vector<float>& ctrl,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
  {
    assert(ctrl.size()==6);
    for (int leg=0;leg<6;leg++)
      {
	std::vector<float> param;
	param.push_back(0); //pinit mot 0                                       
	param.push_back(1); //amp mot 0                                         
	param.push_back(ctrl[leg]); //phase mot 0                               
	param.push_back(0); //Pinit mot 1 & 2                                   
	param.push_back(0.3); //amp Mot 1 & 2                                   
	if(leg<3)
	  {
	    param.push_back(ctrl[leg]+0.25); //phase mot 1                      
	    param.push_back(ctrl[leg]+0.25); // phase mot 2                     
	  }
	else
	  {
	    param.push_back(ctrl[leg]-0.25); //phase mot 1                      
	    param.push_back(ctrl[leg]-0.25); // phase mot 2                     
	  }
	if(param[param.size()-1]==1.25)
	  {
	    param[param.size()-1]=0.25;
	    param[param.size()-2]=0.25;
	  }
	if(param[param.size()-1]==-0.25)
	  {
	    param[param.size()-1]=0.75;
	    param[param.size()-2]=0.75;
	  }
	_legsParams.push_back(param);
	
	
      }
     

  }

  void moveRobot(robot_t& robot, float t);
  std::vector<int> get_pos_dyna(float t);
  std::vector<int> get_speeds_dyna( );
  std::vector<bool> get_directions_dyna( );


};

#endif
