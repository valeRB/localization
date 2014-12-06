#include "ros/ros.h"
#include "math.h"
#include "ras_arduino_msgs/Odometry.h"
#include "robot_msgs/IrTransformMsg.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "robot_msgs/IrTransformMsg.h"
#include "tf/transform_listener.h"


class Localize
{
public:
    ros::NodeHandle n;
    ros::Subscriber odometry_subscriber;
    //ros::Subscriber map_subscriber;
    ros::Subscriber sensor_subscriber;
    ros::Publisher map_publisher;
    ros::Publisher pose_publisher;
    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    std::vector<signed char> grid_map, loc_map;
    geometry_msgs::PoseStamped poseStamp_msg;
    robot_msgs::IrTransformMsg sensor_msg;

    double x_t_ir, y_t_ir, theta_t_ir;
    double x_t_odom, y_t_odom, theta_t_odom;
    double x_prime, y_prime, theta_prime;
    double resolution;
    int center_x, center_y, height_robot, width_robot, width_map, cellNumber;
    double x_pose_cell, y_pose_cell, prev_x_pose_cell, prev_y_pose_cell;
    double b, r, sampleTime;

    Localize()
    {
        n = ros::NodeHandle();
    }

    ~Localize()
    {}
    void init()
    {
        odometry_subscriber = n.subscribe("/arduino/odometry", 1, &Localize::odometryCallback,this);

        sensor_subscriber = n.subscribe("/transformed_ir_points",1, &Localize::sensorCallback,this);
        map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/loc/savedmap");
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/loc/pose", 1);


        cellNumber = 500*500;
        loc_map = std::vector<signed char>(cellNumber,-1);
        resolution = 0.02; //[m]
        center_x = 250; //[cell]
        center_y = 250;
        height_robot = 10;
        width_robot = 10;
        width_map = 500;
        b=0.21;
        r=0.05;
        x_prime = 0;
        y_prime = 0;
        theta_prime = 0;
        sampleTime = 0.05;
    }

    void getMap()
    {

        rosbag::Bag bag;
        bag.open("map_test.bag", rosbag::bagmode::Read);

        //std::vector<std::string> topics;
        //topics.push_back(std::string("chatter"));
        //topics.push_back(std::string("numbers"));

        rosbag::View view(bag, rosbag::TopicQuery("/gridmap"));

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            ROS_INFO("in weird foreach");
            map_msg = m.instantiate<nav_msgs::OccupancyGrid>();
        }

        bag.close();
    }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &enc_msg)
    {
        double delta_enc1 = enc_msg->delta_encoder1;
        double delta_enc2 = enc_msg->delta_encoder2;

        double AngVelLeft =(delta_enc2 * (M_PI/180))/sampleTime;
        double AngVelRight =(delta_enc1 * (M_PI/180))/sampleTime;

        // Pose estimate according to formulas from file of Lab3
        x_t_odom = ((-(r*sin(theta_prime))/2.0)*AngVelLeft + (-(r*sin(theta_prime))/2.0)*AngVelRight)*sampleTime;
        y_t_odom = (((r*cos(theta_prime))/2.0)*AngVelLeft + ((r*cos(theta_prime))/2.0)*AngVelRight)*sampleTime;

        theta_t_odom = ((-r/b)*AngVelLeft + (r/b)*AngVelRight)*sampleTime;
        theta_t_odom = angleBoundaries(theta_t_odom);

    }

    void poseUpdate(double x_t, double y_t, double theta_t)
    {

        x_prime = x_prime + x_t;
        y_prime = y_prime + y_t;
        theta_prime = theta_prime + theta_t;
        theta_prime = angleBoundaries(theta_prime);
        //ROS_INFO("theta_prime %f", theta_prime);
        ROS_INFO("x_prime %f", x_prime);
        ROS_INFO("y_prime %f", y_prime);
        ROS_INFO("theta_prime %f", theta_prime);
        //Publish message

        poseStamp_msg.pose.position.x = x_prime + 5.0;
        poseStamp_msg.pose.position.y = y_prime + 5.0;
        poseStamp_msg.pose.position.z = 0;

        tf::Quaternion q;
        q.setEuler(0.0, 0.0, M_PI_2 + theta_prime);
        tf::quaternionTFToMsg(q, poseStamp_msg.pose.orientation);
        pose_marker_publisher.publish(poseStamp_msg);

    }


    void sensorCallback(const robot_msgs::IrTransformMsg &msg)
    {
        sensor_msg = msg;
    }
/*
    void updateLocOdom()
    {
        //if we get good measurements and check for theta conditions we will update pose with either
        // of the two estimates of x_t,y_t,theta_t.
    }*/
    void publishMap()
    {
        map_publisher.publish(map_msg);
    }

private:

    double angleBoundaries(double theta)
    {
        if (theta > 0)
            theta = fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
        else
            theta = fmod(theta - M_PI, 2.0 * M_PI) + M_PI;
        return theta;
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");

    Localize loc;
    loc.init();

    ros::Rate loop_rate(20.0);

    while(loc.n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
