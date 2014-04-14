#ifndef         SERVO_HH_
# define        SERVO_HH_

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "object.hh"

#define DEFAULT_STOP (1 * M_PI / 2.1f)
 //#define DEFAULT_STOP (dInfinity)
#define DEFAULT_FMAX (1500)

namespace ode
{
  class Servo
  {
    public:
      typedef boost::shared_ptr<Servo> ptr_t;
      enum { DIHEDRAL = 0, SWEEP = 1, TWIST = 2 };
      enum { M_POS = 0, M_VEL };
       /// Object order can matter !
       /// (object1 is the fixed one, object2 the moving one => object2
       /// has the pointer to the servo)
      Servo(Environment & env,
            const Eigen::Vector3d & anchor,
            Object & o1, Object & o2,
            int mode = M_POS,bool init=true) :
        _env(env),
        _anchor(anchor),
        _o1(o1), _o2(o2),
        _angles(Eigen::Vector3d::Zero()),
        _passive(false),
        _power(0),
        _torque(0),
        _vrot(0),
        _mode(mode),
        _vel(Eigen::Vector3d::Zero()),
        _lim_min(Eigen::Vector3d::Constant(-DEFAULT_STOP)),
        _lim_max(Eigen::Vector3d::Constant(DEFAULT_STOP)),
        _blocked(false),
        _p(1.0),
        _offset(Eigen::Vector3d::Zero())
      {
	if( init)
	  _init();
      }

      ~Servo()
      {
        dJointDestroy(_ball);
        dJointDestroy(_amotor);
      }
    Servo(const Servo &s, Environment & env, Object & o1, Object & o2,bool init=true);
      virtual ptr_t clone(Environment& env, Object& o1, Object& o2) const
      { return ptr_t(new Servo(*this, env, o1, o2)); }
      void next_step(float dt);
       /// desired angle
      virtual void set_angle(unsigned i, float v)
      {
        assert(_mode == M_POS);
        v = std::min((double)v, DEFAULT_STOP);
        v = std::max((double)v, -DEFAULT_STOP);
        _angles(i) = v;
      }
      void set_vel(unsigned i, float v)
      {
        assert(_mode == M_VEL);
        _vel(i) = v;
      }
      virtual void set_lim(unsigned i, float min, float max)
      {
        assert(i < 3);
        _lim_min[i] = min;
        _lim_max[i] = max;
      }
      void set_offset(const Eigen::Vector3d& offset) { _offset = offset; }
      const Eigen::Vector3d& get_offset() const { return _offset; }
      void set_mode(int m)
      {
        assert(m == M_POS || m == M_VEL);
        _mode = m;
      }
       /// real angle + offset
      float get_angle(unsigned i) const
      {
        return dJointGetAMotorAngle(_amotor, i) + _offset[i];
      }
      const Eigen::Vector3d& get_anchor() const
      {
        return _anchor;
      }
      void set_anchor(Eigen::Vector3d anchor)
      {
        _anchor = anchor;
        dJointSetBallAnchor(_ball, _anchor.x(), _anchor.y(), _anchor.z());
      }
      virtual void set_passive()
      {
        _passive = false;
        dJointSetAMotorParam(_amotor, dParamFMax, 0);
        dJointSetAMotorParam(_amotor, dParamFMax2, 0);
        dJointSetAMotorParam(_amotor, dParamFMax3, 0);
      }
    virtual void set_axis(size_t a, const Eigen::VectorXd& ax)
      {
	
        dJointSetAMotorAxis(_amotor, a, 1, ax.x(), ax.y(), ax.z());
      }
      void set_blocked(bool b) { _blocked = b; }
      bool get_blocked() const { return _blocked; }
      void set_p(float p) { _p = p; }
      const Object& get_o1() const { return _o1; }
      const Object& get_o2() const { return _o2; }
      float get_power() const { return _power; }
      float get_torque() const { return _torque; }
      float get_vrot() const { return _vrot; }
  protected:
    void _init();

       // http://ode.org/cgi-bin/wiki.pl?ServoSimulation
       // simple P loop
      virtual void _asserv(unsigned i, float dt);
      void _forced_movement(unsigned i)
      {
        dJointSetAMotorParam(_amotor, dParamLoStop, _angles(0));
        dJointSetAMotorParam(_amotor, dParamHiStop, _angles(0));
        dJointSetAMotorParam(_amotor, dParamLoStop2, _angles(1));
        dJointSetAMotorParam(_amotor, dParamHiStop2, _angles(1));
        dJointSetAMotorParam(_amotor, dParamLoStop3, _angles(2));
        dJointSetAMotorParam(_amotor, dParamHiStop3, _angles(2));
      }
      int _vel_selector(int i)
      {
        switch (i)
        {
        case 0:
          return dParamVel;
        case 1:
          return dParamVel2;
        case 2:
          return dParamVel3;
        default:
          assert(0);
        }
        return dParamVel;
      }
       // attributes
      Environment& _env;
      Eigen::Vector3d _anchor;
      Object& _o1;
      Object& _o2;
      dJointID _ball;
      dJointID _amotor;
      Eigen::Vector3d _angles;
      bool _passive;
      dJointFeedback _feedback;
      float _power;
      float _torque;
      float _vrot;
      int _mode;
      Eigen::Vector3d _vel;
      Eigen::Vector3d _lim_min, _lim_max;
      bool _blocked;
      float _p;
      Eigen::Vector3d _offset;
  };
}


#endif      /* !SERVO_HH_ */
