#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/JointState.h>
#include <vector>

ros::Publisher CAN_pub;
ros::Subscriber js_sub;
uint8_t DummyMsg[] = {7, 6, 5, 4, 3, 2, 1, 0};
XmlRpc::XmlRpcValue joint_CAN_ID;


//////////////////////////
can_msgs::Frame poyo_msg[3];
//////////////////////////

can_msgs::Frame create_CAN_message(uint32_t _id, uint8_t *_data){
    can_msgs::Frame msg;
    msg.header.stamp = ros::Time::now();
    msg.id = _id;
    msg.dlc = 8;
    msg.is_error = false;
    msg.is_extended = false;
    msg.is_rtr = false;
    memcpy(&msg.data, _data, 8);
    return msg;
}

void Callback(const sensor_msgs::JointState js_msg){
    int16_t joint_quantity = (int)js_msg.name.size();
    for (uint16_t i = 0; i < joint_quantity; i++)
    {
        int32_t id = static_cast<std::int32_t>(joint_CAN_ID[js_msg.name[i]]);
        float pos_val = (float)js_msg.position[i];
        uint8_t data[] = {1, 0, 0, 0, 0, 0, 0, 0};
        memcpy(&data[4], &pos_val, 4);
        ROS_INFO("Quantity: %d,  ID: %x,  Pos: %f",joint_quantity, id, pos_val);
    
        can_msgs::Frame CAN_msg = create_CAN_message(id, data);
        // CAN_pub.publish(CAN_msg);

//////////////////////
        poyo_msg[i] = CAN_msg;
//////////////////////

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_states_to_CAN_Node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("joint_CAN_ID", joint_CAN_ID);
    CAN_pub = nh.advertise<can_msgs::Frame>("CAN_robot_command", 1000);
    js_sub = nh.subscribe("joint_states", 1000, Callback);
    // ros::spin();

////////////////////
    ros::Rate loop_rate(100);
    ros::Rate rate(1000);
    while (ros::ok())
    {
        CAN_pub.publish(poyo_msg[0]);
        rate.sleep();
        CAN_pub.publish(poyo_msg[1]);
        rate.sleep();
        CAN_pub.publish(poyo_msg[2]);
        loop_rate.sleep();
        ros::spinOnce();
    }
////////////////////

    return 0;
}