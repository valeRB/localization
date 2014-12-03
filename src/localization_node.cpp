#include "ros/ros.h"
#include "math.h"
#include "ras_arduino_msgs/Odometry.h"
#include "robot_msgs/IrTransformMsg.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>


class Localize
{
public:
    ros::NodeHandle n;
    ros::Subscriber odometry_subscriber;
    //ros::Subscriber map_subscriber;
    ros::Subscriber sensor_subscriber;
    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    std::vector<signed char> grid_map, loc_map;

    robot_msgs::IrTransformMsg sensor_msg;

    double x_t, y_t, theta_t;
    double resolution;
    int center_x, center_y, height_robot, width_robot, width_map, cellNumber;
    double x_pose_cell, y_pose_cell, prev_x_pose_cell, prev_y_pose_cell;

    Localize()
    {
        n = ros::NodeHandle();
    }

    ~Localize()
    {}
    void init()
    {
        odometry_subscriber = n.subscribe("/arduino/odometry", 1, &Localize::odometryCallback,this);
        //map_subscriber = n.subscribe("/gridmap", 1, &Localize::mapCallback, this);
        sensor_subscriber = n.subscribe("/transformed_ir_points",1, &Localize::sensorCallback,this);
        //map_publisher = n.advertise
        cellNumber = 500*500;
        loc_map = std::vector<signed char>(cellNumber,-1);
        resolution = 0.02; //[m]
        center_x = 250; //[cell]
        center_y = 250;
        height_robot = 10;
        width_robot = 10;
        width_map = 500;
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
            nav_msgs::OccupancyGrid::ConstPtr map_msg = m.instantiate<nav_msgs::OccupancyGrid>();
        }

        bag.close();
    }

    void odometryCallback(const ras_arduino_msgs::Odometry::ConstPtr &pose_msg)
    {
        x_t = pose_msg->x;
        y_t = pose_msg->y;
        theta_t = pose_msg->theta;
    }

//    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
//    {
//        cost_map = map_msg->data;
//    }

    void sensorCallback(const robot_msgs::IrTransformMsg &msg)
    {
        sensor_msg = msg;
    }

    void updateLocOdom()
    {
        x_pose_cell = floor((center_x + x_t)/resolution);
        y_pose_cell = floor((center_y + y_t)/resolution);

        if((prev_x_pose_cell != x_pose_cell) || (prev_y_pose_cell != y_pose_cell))
        {
            for(int i = x_pose_cell-(width_robot/2); i <= (x_pose_cell+(width_robot/2)); i++)
            {
                for(int j = y_pose_cell-floor(height_robot/2); j <= (y_pose_cell+floor(height_robot/2)); j++)
                {
                    loc_map[i+width_map*j] = 110; //green
                }
            }
            prev_x_pose_cell = x_pose_cell;
            prev_y_pose_cell = y_pose_cell;
        }

    }
    void publishMap()
    {

    }

private:

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
