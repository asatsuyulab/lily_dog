#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <lily_dog/leg_robot_hw.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_robot");

    leg_robot leg;
    controller_manager::ControllerManager cm(&leg, leg.nh);
    
    ros::Rate rate(100);
    ros::AsyncSpinner spinner(1);

    spinner.start();
    while(ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration dt = ros::Duration(0.01);
        leg.read(now, dt);
        cm.update(now, dt);
        leg.write(now, dt);
        rate.sleep();
    }
    spinner.stop();

    return 0;
}
