#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<nav_msgs/Odometry.h>

tf::Quaternion base2odom_quad(0,0,0,1);
tf::Vector3 base2odom_pos(0,0,0);
tf::TransformBroadcaster *broadcaster;
tf::TransformListener *listener;
void groundTruePosCallback(const nav_msgs::Odometry::ConstPtr& msg){
    float x=msg->pose.pose.orientation.x;
    float y=msg->pose.pose.orientation.y;
    float z=msg->pose.pose.orientation.z;
    float w=msg->pose.pose.orientation.w;
    float pos_x=msg->pose.pose.position.x;
    float pos_y=msg->pose.pose.position.y;
    float pos_z=msg->pose.pose.position.z;
    ros::Time current_time=msg->header.stamp;
    base2odom_quad=tf::Quaternion(x,y,z,w);
    base2odom_pos=tf::Vector3(pos_x,pos_y,pos_z);
    tf::Transform base2odom=tf::Transform(base2odom_quad,base2odom_pos);
    broadcaster->sendTransform(
                tf::StampedTransform(
                    base2odom,current_time,"odom","base_footprint"));
}

int main(int argc, char** argv){
    ros::init(argc,argv,"odom_tf_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("base_pose_ground_truth",100,groundTruePosCallback);
    broadcaster=new tf::TransformBroadcaster();
    listener=new tf::TransformListener();
    ros::spin();
}
