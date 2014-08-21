#include "quadruped.hh"

#include "ode/box.hh"
#include "ode/capped_cyl.hh"

using namespace ode;
using namespace Eigen;

namespace robot
{
    void Quadruped :: _build(Environment& env, const Vector3d& pos)
    {
        static const double body_mass = 1;
        static const double body_length = 0.75;
        static const double body_width = 0.5;
        static const double body_height = 0.2;
        static const double leg_w = 0.05;
        static const double leg_length = 0.5;
        static const double leg_dist = 0.2;
        static const double leg_mass = 0.2;

        _main_body = Object::ptr_t
                     (new Box(env, pos + Vector3d(0, 0, leg_length),
                              body_mass, body_length, body_width, body_height));
        _bodies.push_back(_main_body);

        for (size_t i = 0; i < 4; ++i)
        {
            int left_right = i > 1 ? -1 : 1;
            int back_front = i % 2 == 0 ? -1 : 1;
            Object::ptr_t l1
            (new CappedCyl(env, pos + Vector3d(back_front * (leg_dist / 2 + leg_length / 2),
                                               left_right * (body_width / 2 + leg_w),
                                               leg_length),
                           leg_mass, leg_w, leg_length));
            l1->set_rotation(0, M_PI / 2.0, 0);
            _bodies.push_back(l1);

            Ax12::ptr_t s1
            (new Ax12(env, pos + Vector3d(back_front * leg_dist / 2,
                                          left_right * (body_width / 2 + leg_w),
                                          leg_length),
                      *_main_body, *l1));
            _servos.push_back(s1);

            Object::ptr_t l11
            (new CappedCyl(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
                                               left_right * (body_width / 2 + leg_w),
                                               leg_length  / 2),
                           leg_mass, leg_w, leg_length));
            _bodies.push_back(l11);
            Ax12::ptr_t s2
            (new Ax12(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
                                          left_right * (body_width / 2 + leg_w),
                                          leg_length),
                      *l1, *l11));
            _servos.push_back(s2);
        }
        for (size_t i = 0; i < _servos.size(); ++i)
            for (size_t j = 0; j < 3; ++j)
                _servos[i]->set_lim(j, -M_PI / 3, M_PI / 3);
    }
}
