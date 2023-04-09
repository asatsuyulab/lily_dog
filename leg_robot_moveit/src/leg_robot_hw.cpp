#include <leg_robot_moveit/leg_robot_hw.h>

leg_robot::leg_robot(ros::NodeHandle _nh)
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

    motor_commander = _nh.advertise<can_msgs::Frame>("sent_messages", 1000);
    motor_state_reciever = _nh.subscribe("recieved_messages", 10, &leg_robot::recieve_callback, this);

    for(int i; i<3; i++)
        _pos[i] = _vel[i] = _eff[i] = 0;
}

void leg_robot::recieve_callback(const can_msgs::Frame& msg)
{
    float val;
    memcpy(&val, &msg.data[4], 4);
    switch (msg.id)
    {
    case SHOULDER1_POS_ACTUAL_ID:
        _pos[0] = val;
        break;
        
    case SHOULDER1_VEL_ACTUAL_ID:
        _vel[0] = val;
        break;

    case SHOULDER1_EFF_ACTUAL_ID:
        _eff[0] = val;
        break;

    case SHOULDER2_POS_ACTUAL_ID:
        _pos[1] = val;
        break;
        
    case SHOULDER2_VEL_ACTUAL_ID:
        _vel[1] = val;
        break;

    case SHOULDER2_EFF_ACTUAL_ID:
        _eff[1] = val;
        break;
    
    case KNEE_POS_ACTUAL_ID:
        _pos[2] = val;
        break;
    
    case KNEE_VEL_ACTUAL_ID:
        _vel[2] = val;
        break;
    
    case KNEE_EFF_ACTUAL_ID:
        _eff[2] = val;
        break;
    
    default:
        break;
    }
}

void leg_robot::read(const ros::Time &time, const ros::Duration &period)
{
    for(int i=0; i<3; i++)
        if(!isnan(_cmd[i]))
            _pos[i] = _cmd[i];
}

void leg_robot::write(const ros::Time &time, const ros::Duration &period)
{
    can_msgs::Frame msg;
    msg.header.stamp = ros::Time::now();
    msg.dlc = 8;
    msg.is_error = false;
    msg.is_extended = false;
    msg.is_rtr = false;
    int ID[3] = {SHOULDER1_POS_COMMAND_ID, SHOULDER2_POS_COMMAND_ID, KNEE_POS_COMMAND_ID};
    for(int i=0; i<3; i++)
    {
        float val = _cmd[i];
        msg.id = ID[i];
        memcpy(&msg.data[4], &val, 4);
        msg.data[0] = 1;
        motor_commander.publish(msg);
    }
}

ros::Time leg_robot::get_time()
{
    return ros::Time::now();
}

ros::Duration leg_robot::get_period()
{
    return ros::Duration(0.01);
}