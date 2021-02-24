#include <leg_robot_hw.h>

leg_robot::leg_robot()
{
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_shoulder1( "shoulder1_joint", &_pos[0], &_vel[0], &_eff[0] );
    jnt_state_interface.registerHandle(state_handle_shoulder1);

    hardware_interface::JointStateHandle state_handle_shoulder2( "shoulder2_joint", &_pos[1], &_vel[1], &_eff[1] );
    jnt_state_interface.registerHandle(state_handle_shoulder2);
    
    hardware_interface::JointStateHandle state_handle_knee( "knee_joint", &_pos[2], &_vel[2], &_eff[2] );
    jnt_state_interface.registerHandle(state_handle_knee);

    registerInterface( &jnt_state_interface );

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_shoulder1( jnt_state_interface.getHandle("shoulder1_joint"), &_cmd[0] );
    jnt_pos_interface.registerHandle(pos_handle_shoulder1);

    hardware_interface::JointHandle pos_handle_shoulder2( jnt_state_interface.getHandle("shoulder2_joint"), &_cmd[1] );
    jnt_pos_interface.registerHandle(pos_handle_shoulder2);

    hardware_interface::JointHandle pos_handle_knee( jnt_state_interface.getHandle("knee_joint"), &_cmd[2] );
    jnt_pos_interface.registerHandle(pos_handle_knee);

    registerInterface( &jnt_pos_interface );
}

void read (const ros::Time &time, const ros::Duration &period)
{

}

void write (const ros::Time &time, const ros::Duration &period)
{
    
}