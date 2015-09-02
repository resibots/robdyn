#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment_hexa.hh"
#include "robot/hexapod.hh"
#include "ode/box.hh"
#include "renderer/osg_visitor.hh"
#include "controllerPhase6D.hpp"
#define PERIOD 2.0f

float func(float x)
{
    return tanh(sin(x) * 4);
}

typedef boost::shared_ptr<robot::Hexapod> robot_t;

int main()
{
    static const float step = 0.015;
    std::vector< float > ctrl;
    /*
    // marche hexapode
//leg 0
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.0); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.25); // Phase mot 1
    ctrl.push_back(0.25); // Phase mot 2
//leg 1
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.5); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.75); // Phase mot 1
    ctrl.push_back(0.75); // Phase mot 2
//leg 2
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.0); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.25); // Phase mot 1
    ctrl.push_back(0.25); // Phase mot 2
//leg 3
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.0); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.75); // Phase mot 1
    ctrl.push_back(0.75); // Phase mot 2
//leg 4
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.5); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.25); // Phase mot 1
    ctrl.push_back(0.25); // Phase mot 2
//leg 5
    ctrl.push_back(0.5); // Pinit mot0
    ctrl.push_back(0.5); // Amp mot0
    ctrl.push_back(0.0); // Phase mot0
    ctrl.push_back(0.25); // Pinit mot 1 & 2
    ctrl.push_back(0.25); // Amp mot 1 & 2
    ctrl.push_back(0.75); // Phase mot 1
    ctrl.push_back(0.75); // Phase mot 2
    */
    /*
    // marche quadripede
    //leg 0
        ctrl.push_back(0.25); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.125); // Pinit mot 1 & 2
        ctrl.push_back(0.25); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.75); // Phase mot 2
    //leg 1
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2
    //leg 2
        ctrl.push_back(0.75); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.125); // Pinit mot 1 & 2
        ctrl.push_back(0.25); // Amp mot 1 & 2
        ctrl.push_back(0.5); // Phase mot 1
        ctrl.push_back(0.75); // Phase mot 2


    //leg 3
        ctrl.push_back(0.25); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.125); // Pinit mot 1 & 2
        ctrl.push_back(0.25); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.25); // Phase mot 2



    //leg 4
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2
    //leg 5
        ctrl.push_back(0.75); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.125); // Pinit mot 1 & 2
        ctrl.push_back(0.25); // Amp mot 1 & 2
        ctrl.push_back(0.5); // Phase mot 1
        ctrl.push_back(0.25); // Phase mot 2

    */

     // marche TEST

    /*
        //leg 0
            ctrl.push_back(0.625); // Pinit mot0
            ctrl.push_back(1 ); // Amp mot0
            ctrl.push_back(0.0); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.25); // Amp mot 1 & 2
            ctrl.push_back(0.25); // Phase mot 1
            ctrl.push_back(0.25); // Phase mot 2
        //leg 1
            ctrl.push_back(0.5); // Pinit mot0
            ctrl.push_back(1); // Amp mot0
            ctrl.push_back(0.5); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.225); // Amp mot 1 & 2
            ctrl.push_back(0.75); // Phase mot 1
            ctrl.push_back(0.75); // Phase mot 2
        //leg 2
            ctrl.push_back(0.375); // Pinit mot0
            ctrl.push_back(1); // Amp mot0
            ctrl.push_back(0.0); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.25); // Amp mot 1 & 2
            ctrl.push_back(0.25); // Phase mot 1
            ctrl.push_back(0.25); // Phase mot 2
        //leg 3
            ctrl.push_back(0.625); // Pinit mot0
            ctrl.push_back(1); // Amp mot0
            ctrl.push_back(0.0); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.25); // Amp mot 1 & 2
            ctrl.push_back(0.75); // Phase mot 1
            ctrl.push_back(0.75); // Phase mot 2
        //leg 4
            ctrl.push_back(0.5); // Pinit mot0
            ctrl.push_back(1); // Amp mot0
            ctrl.push_back(0.5); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.25); // Amp mot 1 & 2
            ctrl.push_back(0.25); // Phase mot 1
            ctrl.push_back(0.25); // Phase mot 2
        //leg 5
            ctrl.push_back(0.375); // Pinit mot0
            ctrl.push_back(1); // Amp mot0
            ctrl.push_back(0.0); // Phase mot0
            ctrl.push_back(0.25); // Pinit mot 1 & 2
            ctrl.push_back(0.25); // Amp mot 1 & 2
            ctrl.push_back(0.75); // Phase mot 1
            ctrl.push_back(0.75); // Phase mot 2



    */


    /*
    // marche bipede
    //leg 0
        ctrl.push_back(0.25); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.375); // Pinit mot 1 & 2
        ctrl.push_back(0.375); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.75); // Phase mot 2
    //leg 1
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2
    //leg 2
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2


    //leg 3
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2



    //leg 4
        ctrl.push_back(0.5); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.0); // Pinit mot 1 & 2
        ctrl.push_back(0.0); // Amp mot 1 & 2
        ctrl.push_back(0.0); // Phase mot 1
        ctrl.push_back(0.0); // Phase mot 2
    //leg 5
        ctrl.push_back(0.75); // Pinit mot0
        ctrl.push_back(0.0); // Amp mot0
        ctrl.push_back(0.0); // Phase mot0
        ctrl.push_back(0.375); // Pinit mot 1 & 2
        ctrl.push_back(0.375); // Amp mot 1 & 2
        ctrl.push_back(0.5); // Phase mot 1
        ctrl.push_back(0.25); // Phase mot 2
    */
    std::vector< int > brokenLegs;
    /* brokenLegs.push_back(0);
    brokenLegs.push_back(1);
    brokenLegs.push_back(2);
    brokenLegs.push_back(3);
    brokenLegs.push_back(4);
    brokenLegs.push_back(5);
    */
    dInitODE();

    ctrl.push_back(0.25);
    ctrl.push_back(0);
    ctrl.push_back(0.5);
    ctrl.push_back(0.5);
    ctrl.push_back(0);
    ctrl.push_back(0.25);

    renderer::OsgVisitor v;//(renderer::OsgVisitor::FIXED);

    boost::shared_ptr<ode::Environment_hexa> env2(new ode::Environment_hexa());
    boost::shared_ptr<ode::Environment_hexa> env(new ode::Environment_hexa());
    robot_t rob2 = robot_t(new robot::Hexapod(*env2, Eigen::Vector3d(0, 0, 0.1),brokenLegs));

    ControllerPhase controller(ctrl,brokenLegs);

    //    rob2->accept(v);
    //controller.moveRobot(rob,0);
    // low gravity to slow things down (eq. smaller timestep?)
    env2->set_gravity(0, 0, -9.81);
    bool stabilized = false;
    int stab = 0;
    for (size_t s = 0; s < 2000 && !stabilized; ++s)
    {
        Eigen::Vector3d prev_pos = rob2->pos();
        rob2->next_step(step);
        env2->next_step(step);
        //v.update();
        if ((rob2->pos() - prev_pos).norm() < 1e-5)
            stab++;
        else
            stab = 0;
        if (stab > 100)
            stabilized = true;
	//	usleep(10000);
    }
    env2->set_gravity(0, 0, -9.81);
    // assert(stabilized);



   robot_t rob = rob2->clone(*env);
   rob->accept(v);
   //v.enable_dump("frame");
    struct timeval timev_init;  // Initial absolute time (static)
    struct timeval timev_diff;  // Previous tick absolute time
    struct timeval timev_cur;   // Current absolute time
    Eigen::Vector3d prev_pos = rob->pos();

    gettimeofday(&timev_init, NULL);

    float t=0;
    while (!v.done() && t<10 )
      {

	//for(int i=0;i< rob->servos().size();i++)
	// rob->servos()[i]->set_angle(ode::Servo::DIHEDRAL, cos(t));


	//rob->servos()[0]->set_angle(ode::Servo::DIHEDRAL, cos(2*M_PI*t));
	//rob->servos()[4]->set_angle(ode::Servo::DIHEDRAL, cos(2*M_PI*t));
	//rob->servos()[8]->set_angle(ode::Servo::DIHEDRAL, cos(2*M_PI*t));


	controller.moveRobot(rob,t);

	rob->next_step(step);
	env->next_step(step);
	 v.update();
	t += step;
	usleep(10000);

      }

    gettimeofday(&timev_cur, NULL);
    timersub(&timev_cur, &timev_init, &timev_diff);

    std::cout<<"time duration "<<timev_diff.tv_sec<< "seconde et "<< timev_diff.tv_usec<< " micro secondes"<<std::endl;

    Eigen::Vector3d next_pos = rob->pos();
    //_covered_distance = fabs(next_pos[0] - prev_pos[0]);
    std::cout<<"dist"<<sqrt((next_pos[0] - prev_pos[0])*(next_pos[0] - prev_pos[0])+(next_pos[1] - prev_pos[1])*(next_pos[1] - prev_pos[1]))<<std::endl;



    return 0;
}
