#ifndef SERVODYNAMIXEL_HH_
#define SERVODYNAMIXEL_HH_

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "object.hh"

#include "servo.hh"

namespace ode
{



  template<int fmax_t> // fmax_t is 10 times the actual fmax !!
  class ServoDynamixel : public Servo
  {
  public:
    typedef boost::shared_ptr<ServoDynamixel<fmax_t> > ptr_t;
    BOOST_STATIC_CONSTEXPR float angular_vel = 6.28; //45rpm
    ServoDynamixel(Environment& env,
	 const Eigen::Vector3d& anchor,
	 Object& o1, Object& o2,
	 int mode = M_POS) :
      Servo(env, anchor, o1, o2, mode,false)
    {
      _init_servo_dynamixel();
    }
    ServoDynamixel(const ServoDynamixel& s, Environment& env, Object& o1, Object& o2) :
      Servo(s, env, o1, o2,false)
    {
	  
      _ball = dJointCreateHinge(_env.get_world(), 0);
      dJointAttach(_ball, _o1.get_body(), _o2.get_body());
      //Eigen::Vector3d anchor=s.get_anchor();
      dVector3 anchor;
      dJointGetHingeAnchor(s._ball, anchor);
      _anchor = Eigen::Vector3d(anchor[0], anchor[1], anchor[2]);
      dJointSetHingeAnchor(_ball, _anchor.x(), _anchor.y(), _anchor.z());
	  
      _amotor = dJointCreateAMotor(_env.get_world(), 0);    
      dJointAttach(_amotor, _o1.get_body(), _o2.get_body());
	  
      dJointSetAMotorNumAxes(_amotor, 1);
      dVector3 axis;
	  
      dJointGetHingeAxis(s._ball, axis);
      dJointSetHingeAxis(_ball, axis[0], axis[1], axis[2]);
	  
      dJointGetAMotorAxis(s._amotor, 0, axis);
      dJointSetAMotorAxis(_amotor, 0, 1, axis[0], axis[1], axis[2]);
      dJointGetAMotorAxis(s._amotor, 2, axis);
      dJointSetAMotorAxis(_amotor, 2, 2, axis[0], axis[1], axis[2]);
      // euler
      dJointSetAMotorMode(_amotor, dAMotorEuler);
	  
  
	  
      /* _lim_min = Eigen::Vector3d::Constant(-5 * M_PI / 6.0f);
	 _lim_max = Eigen::Vector3d::Constant(5 * M_PI / 6.0f);*/
      dJointSetAMotorParam(_amotor, dParamLoStop,  _lim_min[0]);
      dJointSetAMotorParam(_amotor, dParamHiStop,  _lim_max[0]);
      dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
      dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
      dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
      dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);
	  
      //      static const double fmax = 1.5;
      dJointSetAMotorParam(_amotor, dParamFMax, fmax_t/10.0);
      dJointSetAMotorParam(_amotor, dParamFMax2, fmax_t/10.0);
      dJointSetAMotorParam(_amotor, dParamFMax3, fmax_t/10.0);
	  
      //vels
      dJointSetAMotorParam(_amotor, dParamVel, 0);
      dJointSetAMotorParam(_amotor, dParamVel2, 0);
      dJointSetAMotorParam(_amotor, dParamVel3, 0);
	  
      for (size_t i = 0; i < 3; ++i)
	{
	  float a = dJointGetAMotorAngle(s._amotor, i);
	  _offset[i] += a;
	}
	  
      //    _angles(maths::_(0, _angles.size() - 1)) = 0; ??
      _angles = Eigen::Vector3d::Zero();
	  
      _o2.add_servo(this);
      _o1.add_servo2(this);
	  
      dJointSetFeedback(_amotor, &_feedback);
	  
    


    }

    boost::shared_ptr<Servo> clone(Environment& env, Object& o1, Object &o2) const
    {
      return ptr_t(new ServoDynamixel<fmax_t>(*this, env, o1, o2));
    }
    void set_lim(unsigned i, float min, float max)
    {
      assert(i < 3);
      _lim_min[i] = min;
      _lim_max[i] = max;
      dJointSetAMotorParam(_amotor, dParamLoStop, _lim_min[0]);
      dJointSetAMotorParam(_amotor, dParamHiStop, _lim_max[0]);
      dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
      dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
      dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
      dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);

    }
    void set_axis(size_t a, const Eigen::VectorXd& ax)
    {
      if(a!=0)
	return;
      dJointSetAMotorAxis(_amotor, 0, 1, ax.x(), ax.y(), ax.z());
      if(ax.x()!=0 ||ax.y()!=0)
	dJointSetAMotorAxis(_amotor, 2, 1, -ax.y(), ax.x(), 0);
      else
	dJointSetAMotorAxis(_amotor, 2, 1, ax.z(), 0, 0);
      dVector3 axis;
      dJointGetAMotorAxis(_amotor,0,axis);
      dJointSetHingeAxis(_ball, axis[0], axis[1], axis[2]);

    }
    /* void set_hinge_axis( const Eigen::VectorXd& ax)
       {
       dJointSetHingeAxis(_ball, ax.x(), ax.y(), ax.z());
       }*/
    void set_angle(unsigned i, float v)
    {
      //	std::cout<<v<<std::endl;
      assert(_mode == M_POS);
      /*v = std::min((double)v, _lim_max[0]);
        v = std::max((double)v, _lim_min[0]);*/

      _angles(i) = v;
    }

  protected:
    virtual void _init_servo_dynamixel()
    {
      _ball = dJointCreateHinge(_env.get_world(), 0);
      dJointAttach(_ball, _o1.get_body(), _o2.get_body());
      dJointSetHingeAnchor(_ball, _anchor.x(), _anchor.y(), _anchor.z());
 
      _amotor = dJointCreateAMotor(_env.get_world(), 0);
      dJointAttach(_amotor, _o1.get_body(), _o2.get_body());

      dJointSetAMotorNumAxes(_amotor, 1);
      //axis
      dJointSetHingeAxis(_ball, 1, 0, 0);
      dJointSetAMotorAxis(_amotor, 0, 1, 1, 0, 0);
      dJointSetAMotorAxis(_amotor, 2, 2, 0, 0, 1);
      // euler
      dJointSetAMotorMode(_amotor, dAMotorEuler);





      /* _lim_min = Eigen::Vector3d::Constant(-5 * M_PI / 6.0f);
	 _lim_max = Eigen::Vector3d::Constant(5 * M_PI / 6.0f);*/
      dJointSetAMotorParam(_amotor, dParamLoStop,  _lim_min[0]);
      dJointSetAMotorParam(_amotor, dParamHiStop,  _lim_max[0]);
      dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
      dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
      dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
      dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);

      //      static const double fmax = 1.5;
      dJointSetAMotorParam(_amotor, dParamFMax, fmax_t/10.0);
      dJointSetAMotorParam(_amotor, dParamFMax2, fmax_t/10.0);
      dJointSetAMotorParam(_amotor, dParamFMax3, fmax_t/10.0);


      //vels
      dJointSetAMotorParam(_amotor, dParamVel, 0);
      dJointSetAMotorParam(_amotor, dParamVel2, 0);
      dJointSetAMotorParam(_amotor, dParamVel3, 0);

      //    _angles(maths::_(0, _angles.size() - 1)) = 0; ??
      _angles = Eigen::Vector3d::Zero();
	  
      _o2.add_servo(this);
      _o1.add_servo2(this);
	  
      dJointSetFeedback(_amotor, &_feedback);




	   
    }
    virtual void _asserv(unsigned i, float dt)
    {
      if (i!=0)
	return;

      set_p(16);
      float pos = dJointGetAMotorAngle(_amotor, i);
      float error = _angles(i) - pos - _offset[i];
      float vel = error * _p;

      //std::cout<<_angles(i)<<" "<<pos<<" "<<_offset[i]<<std::endl;	  
	  
      if (vel>angular_vel)
	{
	  //std::cout<<"max vel"<<std::endl;
	  vel=angular_vel;
	}
	  
      if (vel<-1*angular_vel)
	{
	  //std::cout<<"max vel"<<std::endl;
	  vel=-1*angular_vel;
	}
	  
      if(std::isnan(vel))
	vel=0;
	  
      if(vel<1e-5 && vel>-1e-5)
	{
	  vel=0;
	}
      // std::cout<<vel<<std::endl;
      dJointSetAMotorParam(_amotor, _vel_selector(i), vel);
    }
  };


  typedef ServoDynamixel<15> Mx28;
  typedef ServoDynamixel<40> Mx64;
  typedef ServoDynamixel<55> Mx106;
  typedef ServoDynamixel<110> DoubleMx106;

}


#endif
