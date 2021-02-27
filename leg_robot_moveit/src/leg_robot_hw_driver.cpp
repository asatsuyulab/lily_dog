#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <leg_robot_moveit/leg_robot_hw.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lily_leg");
    ros::NodeHandle nh;

    leg_robot lily_leg(nh);
    controller_manager::ControllerManager cm(&lily_leg, nh);
    
    ros::AsyncSpinner spinner(1);
    ros::Rate rate(100);
    spinner.start();
    while(ros::ok())
    {
        ros::Time now = lily_leg.get_time();
        ros::Duration dt = lily_leg.get_period();
        lily_leg.read(now, dt);
        cm.update(now, dt);
        lily_leg.write(now, dt);
        rate.sleep();
    }
    spinner.stop();

    return 0;
}
