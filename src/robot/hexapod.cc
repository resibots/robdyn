
#include "hexapod.hh"
#include "ode/box.hh"
#include "ode/capped_cyl.hh"
#include "ode/sphere.hh"
#include "ode/motor.hh"
#include "ode/mx28.hh"
#include "ode/ax12.hh"




using namespace ode;
using namespace Eigen;

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
        static const double body_mass = 1.5;
        static const double body_length = 0.20;
        static const double body_width = 0.24;
        static const double body_height =0.04;

        static const double legP1_w = 0.02;
        static const double legP1_length = 0.06;
        static const double legP1_dist = 0.2;
        static const double legP1_mass = 0.020;

        static const double legP2_w = 0.02;
        static const double legP2_length = 0.085;
        static const double legP2_dist = 0.2;
        static const double legP2_mass = 0.180;


        static const double legP3_w = 0.025;
        static const double legP3_length = 0.145;
        static const double legP3_dist = 0.2;
        static const double legP3_mass = 0.060;



        /// creation of robot's body
        _main_body = Object::ptr_t(new Box(env, pos + Vector3d(0, 0, legP3_length+0.01),
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
	    //             float angle = i<3 ? M_PI / 2.0f : -M_PI / 2.0f;// + M_PI / 6;
	    float angle;
            float xStart=0; // selection of the start of the first joint
            float yStart=0;
            switch(i)
            {
                case 0:
                case 5:
                {
                    xStart=i<3 ? 0.06 : -0.06;
                    yStart=0.12;
		    angle=i<3? 3*M_PI/8:-3*M_PI/8;
                    break;
                }

                case 1:
                case 4:
                {
                    xStart=i<3 ? 0.10 : -0.10;
                    yStart=0;
		    angle=i<3? M_PI/2:-M_PI/2;
                    break;
                }

                case 2:
                case 3:
                {
                    xStart=i<3 ? 0.06 : -0.06;
                    yStart=-0.12;
		    angle=i<3? 5*M_PI/8:-5*M_PI/8;
                    break;
                }
            }

             /// first part
             Object::ptr_t l1(
			      /*                 new Box(env, pos
                               + Vector3d(xStart+sin(angle) * (legP1_length / 2),
                                          yStart+cos(angle) * (legP1_length / 2),
                                          legP3_length+0.01),
			 legP1_mass,0.04, legP1_w, legP1_length));

*/

                 new CappedCyl(env, pos
                               + Vector3d(xStart+sin(angle) * (legP1_length / 2),
                                          yStart+cos(angle) * (legP1_length / 2),
                                          legP3_length+0.01),
	                     legP1_mass, legP1_w, legP1_length));

                             
	     l1->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
			      Vector3d(0, 0, 1));

             _bodies.push_back(l1);
	     env.add_leg_object(i,*l1);
             Mx28::ptr_t s1(new Mx28(env, pos +
                                       Vector3d(xStart,
                                                yStart,
                                                /*legP2_length+*/legP3_length+0.01),
                                       *_main_body, *l1));
	    

           
             //s1->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
	     s1->set_axis(0, Vector3d(0,0, 1));
	     s1->set_axis(2, Vector3d(1,0, 0));
	     //s1->set_hinge_axis(Vector3d(0,0, 1));
             //s1->set_axis(2, Vector3d(0, 0, -1));
	    

	    
            s1->set_lim(0, -M_PI/8, M_PI/8 );

             _servos.push_back(s1);

             /// second part
             Object::ptr_t l2(
			      /*                 new Box(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length/2),
                                        yStart+cos(angle) * (legP1_length+legP2_length/2),
                                        legP3_length+0.01),
					legP2_mass,0.04, legP2_w, legP2_length));*/
	                      new CappedCyl(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length/2),
                                        yStart+cos(angle) * (legP1_length+legP2_length/2),
                                        legP3_length+0.01),
                               legP2_mass, legP2_w, legP2_length));
	     
             /*l2->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
	       Vector3d(sin(-angle), -cos(-angle), 0));*/

	   
    l2->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
		     Vector3d(0, 0, 1));
	

	      _bodies.push_back(l2);

	     env.add_leg_object(i,*l2);

             Mx28::ptr_t s2(new Mx28(env, pos +
                                       Vector3d(xStart+sin(angle) * (legP1_length),
                                                yStart+cos(angle) * (legP1_length),
                                                legP3_length+0.01),
                                       *l1, *l2));
	     s2->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
	     s2->set_axis(2, Vector3d(sin(-angle), -cos(-angle), 0));

	 
	     //s2->set_axis(0, Vector3d(0, 1, 0));
             //s2->set_axis(2, Vector3d(0, 0, 1));
            
	    
	        
            s2->set_lim(0, -M_PI/4, M_PI/4 );
             _servos.push_back(s2);


             /// third part
             Object::ptr_t l3(
                 new CappedCyl(env, pos +
                               Vector3d(xStart+sin(angle) * (legP1_length+legP2_length),
                                        yStart+cos(angle) * (legP1_length+legP2_length),
                                        legP3_length/2+0.01),
                               legP3_mass, legP3_w, legP3_length));

             //l3->set_rotation(Vector3d(cos(-angle), sin(-angle), 0),
	     //               Vector3d(sin(-angle), -cos(-angle), 0));


             _bodies.push_back(l3);
	     env.add_leg_object(i,*l3);
             Mx28::ptr_t s3(new Mx28(env, pos +
                                       Vector3d(xStart+sin(angle) * (legP1_length+legP2_length),
                                                yStart+cos(angle) * (legP1_length+legP2_length),
                                                legP3_length+0.01),
                                       *l2, *l3));
             s3->set_axis(0, Vector3d(cos(-angle), sin(-angle), 0));
	     s3->set_axis(2, Vector3d(sin(-angle), -cos(-angle), 0));


             //s3->set_axis(0, Vector3d(1, 0, 0));
             //s3->set_axis(2, Vector3d(0, 1, 0));


	     //	     dBodyVectorToWorld(l2->get_body(),1,0,0,axis);

	     //s3->set_hinge_axis(Vector3d(axis[0],axis[1],axis[2]));//Vector3d(0,1,0));
	     



            s3->set_lim(0, -M_PI/4, M_PI/4 );
             _servos.push_back(s3);



         }



        for (size_t i = 0; i < _servos.size(); ++i)
            for (size_t j = 0; j < 3; ++j)
                _servos[i]->set_lim(j, -M_PI/2, M_PI/2 );
	/*for (size_t i = 0; i < _servos.size(); i++)
	  {
            for (size_t j = 0; j < 3; ++j)
	      _servos[i]->set_lim(j, 0, 0 );
	      }*/
    }
}

