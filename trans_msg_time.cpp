#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <livox_ros_driver/CustomMsg.h>
void read_msg_from_bag(const std::string &bag_file_path, const std::vector<std::string> &topics){
    rosbag::Bag bag;
    std::string new_bag_path = "/home/qly/project/img_time_minus/src/new1.bag";
    rosbag::Bag out_bag;
    out_bag.open(new_bag_path,rosbag::bagmode::Write);
    bag.open(bag_file_path);
    if(bag.isOpen()==false){
        ROS_ERROR_STREAM("failed to open " << bag_file_path);
        exit(-1);
    }
    printf("get the bag succeed\n");
    rosbag::View view_bag(bag, rosbag::TopicQuery(topics));
    if (view_bag.size() == 0)
    {
        ROS_ERROR_STREAM("error querying bag: " << bag_file_path);
        exit(-1);
    }
    for(rosbag::MessageInstance const m : view_bag){
        std::string topic = m.getTopic();
        if(topic=="/livox/imu"){
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            out_bag.write("/livox/imu",m.getTime(),imu_msg);
        }
        if(topic=="/livox/lidar"){
            livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            out_bag.write("/livox/lidar",m.getTime(),livox_msg);
        }
        if(topic=="/camera/image_color"){
            sensor_msgs::Image::Ptr img_new = m.instantiate<sensor_msgs::Image>();
            long double temp_time = img_new->header.stamp.toSec();
            printf("before change time is %llf\n",temp_time);
            img_new->header.stamp.fromSec(temp_time + 0.1);
            printf("after change time is %llf\n",img_new->header.stamp.toSec());
            out_bag.write("/camera/image_color",m.getTime(),img_new);
        }
    }
    bag.close();
    out_bag.close();
}
int main(int argc, char **argv){
    ros::init(argc, argv,"trans img time");
    ros::NodeHandle nh;
    std::string bag_path;
    std::vector<std::string> topics;
    topics.push_back("/livox/lidar");
    topics.push_back("/livox/imu");
    topics.push_back("/camera/image_color");
    nh.getParam("/trans_msg_time_node/bag_path",bag_path);
    read_msg_from_bag(bag_path,topics);

}