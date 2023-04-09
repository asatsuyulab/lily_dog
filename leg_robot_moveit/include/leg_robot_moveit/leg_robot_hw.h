#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <can_msgs/Frame.h>

#ifndef ID_DEFINITION
#define ID_DEFINITION 1

#define SHOULDER1_POS_COMMAND_ID    0x201
#define SHOULDER2_POS_COMMAND_ID    0x211
#define KNEE_POS_COMMAND_ID         0x221

#define SHOULDER1_POS_ACTUAL_ID     0x206
#define SHOULDER1_VEL_ACTUAL_ID     0x207
#define SHOULDER1_EFF_ACTUAL_ID     0x208

#define SHOULDER2_POS_ACTUAL_ID     0x216
#define SHOULDER2_VEL_ACTUAL_ID     0x217
#define SHOULDER2_EFF_ACTUAL_ID     0x218

#define KNEE_POS_ACTUAL_ID          0x226
#define KNEE_VEL_ACTUAL_ID          0x227
#define KNEE_EFF_ACTUAL_ID          0x228
#endif

class leg_robot : public hardware_interface::RobotHW
{
public:
    leg_robot(ros::NodeHandle _nh);

    void read(const ros::Time &, const ros::Duration &);
    void write(const ros::Time &, const ros::Duration &);
    void recieve_callback(const can_msgs::Frame& msg);
    ros::Time get_time();
    ros::Duration get_period();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    ros::Publisher motor_commander;
    ros::Subscriber motor_state_reciever;
    double _cmd[3];     // for receive command from PosirionJointInterface
    double _pos[3];     // for send position to JointStateInterface
    double _vel[3];     // for send velocity to JointStateInterface
    double _eff[3];     // for send effort to JointStateInterface
};