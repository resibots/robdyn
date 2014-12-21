#ifndef   	AX12_HH_
# define   	AX12_HH_

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "object.hh"

#include "servo.hh"

namespace ode
{
  class Ax12 : public Servo
  {
  public:
    typedef boost::shared_ptr<Ax12> ptr_t;
    BOOST_STATIC_CONSTEXPR float angular_vel = 150.0 / 1024.0 * 11.9;
    Ax12(Environment& env,
	 const Eigen::Vector3d& anchor,
	 Object& o1, Object& o2,
	 int mode = M_POS) :
      Servo(env, anchor, o1, o2, mode)
    { _init_ax12(); }
    Ax12(const Ax12& s, Environment& env, Object& o1, Object& o2) :
      Servo(s, env, o1, o2) { _init_ax12(); }

    boost::shared_ptr<Servo> clone(Environment& env, Object& o1, Object &o2) const
    { return ptr_t(new Ax12(*this, env, o1, o2)); }
  protected:
    void _init_ax12()
    {
      _lim_min = Eigen::Vector3d::Constant(-5 * M_PI / 6.0f);
      _lim_max = Eigen::Vector3d::Constant(5 * M_PI / 6.0f);
      dJointSetAMotorParam(_amotor, dParamLoStop,  _lim_min[0]);
      dJointSetAMotorParam(_amotor, dParamHiStop,  _lim_max[0]);
      dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
      dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
      dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
      dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);

      static const double fmax = 15;
      dJointSetAMotorParam(_amotor, dParamFMax, fmax);
      dJointSetAMotorParam(_amotor, dParamFMax2, fmax);
      dJointSetAMotorParam(_amotor, dParamFMax3, fmax);
    }
    virtual void _asserv(unsigned i, float dt)
    {
      double cur_angle1 = dJointGetAMotorAngle(_amotor, i);
      double error1 = _angles(i) - cur_angle1 - _offset(i);
      double error2 = error1 / (5.0 * M_PI/3.0) * 1024.0;
      int sign = (error1 > 0) * 2 - 1;
      double vel = 0;

      if(fabs(error2) > 1)
	vel = angular_vel * sign;
      else
	vel = 0;
      dJointSetAMotorParam(_amotor, _vel_selector(i), vel);
    }
  };
}


#endif
