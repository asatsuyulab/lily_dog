#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class leg_robot : public hardware_interface::RobotHW
{
public:
    leg_robot();

    ros::NodeHandle nh;
    void read (const ros::Time &, const ros::Duration &);
    void write (const ros::Time &, const ros::Duration &);

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double _cmd[3];     // for receive command from PosirionJointInterface
    double _pos[3];     // for send position to JointStateInterface
    double _vel[3];     // for send velocity to JointStateInterface
    double _eff[3];     // for send effort to JointStateInterface
};