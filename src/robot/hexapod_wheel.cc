#include "hexapod.hh"
#include "ode/box.hh"
#include "ode/capped_cyl.hh"
#include "ode/sphere.hh"
#include "ode/motor.hh"
#include "ode/ax12.hh"



using namespace ode;
USING_PART_OF_NAMESPACE_EIGEN

namespace robot
{
    static int sign(float x)
    {
        return x > 0 ? 1 : -1;
    }
    void Hexapod :: _build(Environment_hexa& env, const Vector3d& pos)
    {

        /// Definition of robot's params
        // length in meter
        // mass in KG
        static const double body_mass = 0.426;
        static const double body_length = 0.20;
        static const double body_width = 0.24;
        static const double body_height = 0.04;

        static const double legP1_w = 0.02;
        static const double legP1_length = 0.06;
        static const double legP1_dist = 0.2;
        static const double legP1_mass = 0.0288;

        static const double legP2_w = 0.02;
        static const double legP2_length = 0.085;
        static const double legP2_dist = 0.2;
        static const double legP2_mass = 0.141;

        static const double legP3_w = 0.025;
        static const double legP3_length = 0.14;
        static const double legP3_dist = 0.2;
        static const double legP3_mass = 0.088;

        static const double legP4_w = 0.02;
        static const double legP4_length = 0.045;
        static const double legP4_dist = 0.2;
        static const double legP4_mass = 0.0615;

        static const double wheel_rad = 0.025;
        static const double wheel_mass = 0.0152;

        /// creation of robot's body
        _main_body = Object::ptr_t(new Box(env, pos + Vector3d(0, 0, legP3_length+wheel_rad),
                                           body_mass, body_length, body_width, body_height));
        _bodies.push_back(_main_body);




         for (size_t i = 0; i <6; ++i) // for each legs
         {
             for(int j=0;j<_brokenLegs.size();j++)
             {
                if(i==_brokenLegs[j])
                {
                    i++;
                    if(_brokenLegs.size()>j+1 && _brokenLegs[j+1]!=i)
                        break;
                }
             }
            if(i>=6)
                return;
             // selecting an angle corresponding to number of the leg
             float angle = i<3 ? M_PI / 4.0f*(1+i) : M_PI / 4.0f*(2+i);// + M_PI / 6;

            float xStart=0; // selection of the start of the first joint
            float yStart=0;
            switch(i)
            {
                case 0:
                case 5:
                {
                    xStart=i<3 ? 0.06 : -0.06;
                    yStart=0.12;
                    break;
                }

                case 1:
                case 4:
                {
                    xStart=i<3 ? 0.10 : -0.10;
                    yStart=0;
                    break;
                }

                case 2:
                case 3:
                {
                    xStart=i<3 ? 0.06 : -0.06;
                    yStart=-0.12;
                    break;
                }
            }

             /// first part
             Object::ptr_t l1(
                 new CappedCyl(env, pos
                               + Vector3d(xStart+sin(angle) * (legP1_length / 2),
                                          yStart+cos(angle) * (legP1_length / 2),
                                          legP3_length+wheel_rad),
                               legP1_mass, legP1_w, legP1_length));

             l1->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
                              Vector3d(0, 0, -1));

             _bodies.push_back(l1);
	     env.add_leg_object(i,*l1);
             Ax12::ptr_t s1(new Ax12(env, pos +
                                       Vector3d(xStart,
                                                yStart,
                                                legP2_length+legP3_length+wheel_rad),
                                       *_main_body, *l1));

             //s1->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
             s1->set_axis(0, Vector3d(0,0, 1));
             //s1->set_axis(2, Vector3d(0, 0, -1));

             _servos.push_back(s1);

             /// second part
             Object::ptr_t l2(
                 new CappedCyl(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length/2),
                                        yStart+cos(angle) * (legP1_length+legP2_length/2),
                                        legP3_length+wheel_rad),
                               legP2_mass, legP2_w, legP2_length));

             /*l2->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
                             Vector3d(sin(-angle), -cos(-angle), 0));*/
                l2->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
                              Vector3d(0, 0, -1));
             _bodies.push_back(l2);
	     env.add_leg_object(i,*l2);


             Ax12::ptr_t s2(new Ax12(env, pos +
                                       Vector3d(xStart+sin(angle) * (legP1_length),
                                                yStart+cos(angle) * (legP1_length),
                                                legP3_length+wheel_rad),
                                       *l1, *l2));
             s2->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
             s2->set_axis(2, Vector3d(sin(-angle), -cos(-angle), 0));

             _servos.push_back(s2);


             /// third part
             Object::ptr_t l3(
                 new CappedCyl(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length),
                                        yStart+cos(angle) * (legP1_length+legP2_length),
                                        legP3_length/2+wheel_rad),
                               legP3_mass, legP3_w, legP3_length));

             l3->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
                             Vector3d(sin(-angle), -cos(-angle), 0));

             _bodies.push_back(l3);
	     env.add_leg_object(i,*l3);
             Ax12::ptr_t s3(new Ax12(env, pos +
                                       Vector3d(xStart+sin(angle) * (legP1_length+legP2_length),
                                                yStart+cos(angle) * (legP1_length+legP2_length),
                                                legP3_length+wheel_rad),
                                       *l2, *l3));
             s3->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
             s3->set_axis(2, Vector3d(sin(-angle), -cos(-angle), 0));


             _servos.push_back(s3);

             /// fourth part
             Object::ptr_t l4(
                 new CappedCyl(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length-legP4_length/2),
                                        yStart+cos(angle) * (legP1_length+legP2_length-legP4_length/2),
                                        wheel_rad),
                               legP4_mass, legP4_w, legP4_length));

             l4->set_rotation(Vector3d(0,0 , -1),
                             Vector3d(cos(-angle), sin(-angle), 0));

             _bodies.push_back(l4);
	     env.add_leg_object(i,*l4);
             Ax12::ptr_t s4(new Ax12(env, pos +
                                       Vector3d(xStart+sin(angle) * (legP1_length+legP2_length+legP3_w),
                                                yStart+cos(angle) * (legP1_length+legP2_length+legP3_w),
                                                wheel_rad),
                                       *l3, *l4));
             // problem
             s4->set_axis(0, Vector3d(0,0,1));
             s4->set_axis(2, Vector3d(sin(-angle), -cos(-angle), 0));

             _servos.push_back(s4);


             /// wheel

            Object::ptr_t wheel
             (new Sphere(env, pos +
                             Vector3d(xStart+sin(angle) * (legP1_length+legP2_length-legP4_length),
                                     yStart+cos(angle) * (legP1_length+legP2_length-legP4_length),
                                     wheel_rad),
                         wheel_mass, wheel_rad));



             _bodies.push_back(wheel);
	     env.add_leg_object(i,*wheel);
             Motor::ptr_t m1
                         (new Motor(env, pos +
                                        Vector3d(xStart+ sin(angle) * ( legP1_length+legP2_length-legP4_length),
                                                yStart+ cos(angle) * ( legP1_length+legP2_length-legP4_length),
                                                wheel_rad),
                                        Vector3d(cos(-angle), sin(-angle), 0),
                                        *l4, *wheel));
             _motors.push_back(m1);

         }



        for (size_t i = 0; i < _servos.size(); ++i)
            for (size_t j = 0; j < 3; ++j)
                _servos[i]->set_lim(j, -M_PI/3, M_PI/3 );
    }
}
