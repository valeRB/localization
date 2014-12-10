#include "ros/ros.h"
#include "math.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_arduino_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
//#include "robot_msgs/IrTransformMsg.h"
#include "tf/transform_listener.h"


class Localize
{
public:
    ros::NodeHandle n;
    ros::Subscriber encoder_subscriber;    
    ros::Subscriber sensor_subscriber;
    ros::Subscriber grid_subscriber;
    ros::Publisher pose_publisher;
    ros::Publisher map_publisher;
    ros::Publisher poseVis_publisher;
    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    std::vector<signed char> grid_map, loc_map;
    geometry_msgs::PoseStamped poseStamp_msg;
    ras_arduino_msgs::ADConverter sensor_msg;
    ras_arduino_msgs::Odometry new_pose;

    double x_t_ir, y_t_ir, theta_t_ir;
    double x_t_odom, y_t_odom, theta_t_odom;
    double x_prime, y_prime, theta_prime;
    double resolution;
    int center_x, center_y, height_robot, width_robot, width_map, cellNumber;
    int x_pose_cell_map, y_pose_cell_map;
    int wall_x, wall_y;
    int x_t_cell, y_t_cell;
    int cell_dist, margin_cell, x_cell, y_cell;
    double center_x_m, center_y_m, dist;
    double x_pose_cell, y_pose_cell, prev_x_pose_cell, prev_y_pose_cell;
    double dist_s1, dist_s2, dist_s3, dist_s4;
    double b, r, sampleTime, robot_radius;
    double eps;
    int wallValue;


    Localize()
    {
        n = ros::NodeHandle();
    }

    ~Localize()
    {}
    void init()
    {
        sensor_subscriber = n.subscribe("/ir_sensor_cm", 1, &Localize::sensorCallback, this);        
        encoder_subscriber = n.subscribe("/arduino/encoders", 1, &Localize::encoderCallback,this);        
        grid_subscriber = n.subscribe("/gridmap", 1, &Localize::getMapCallback, this);        
        map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/loc/savedmap",1);
        pose_publisher = n.advertise<ras_arduino_msgs::Odometry>("/loc/pose", 1);
        poseVis_publisher = n.advertise<geometry_msgs::PoseStamped>("/poseVis", 1);

        cellNumber = 500*500;
        //loc_map = std::vector<signed char>(cellNumber);
        resolution = 0.02; //[m]
        center_x = 250; //[cell]        
        center_y = 250;
        center_x_m = 5.0; //[m]
        center_y_m = 5.0; //[m]
        height_robot = 10;
        width_robot = 10;
        width_map = 500;
        robot_radius = 0.065; //[m]
        b=0.21;
        r=0.05;
        loc_map = std::vector<signed char>(cellNumber);
        wallValue = -106;

        sampleTime = 0.05;
        eps = 8*(M_PI/180.0);//equivalent to 8 deg; given in [rad]
        poseStamp_msg.header.frame_id = "map";
        poseStamp_msg.header.stamp = ros::Time(0);
    }   

    void getInitialPose()
    {
        x_prime = 0;
        y_prime = 0;
        theta_prime = 0;
    }

    void getMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        map_msg = msg;
        loc_map = map_msg->data;

        updateLocalization();
        //publishMap();
    }

    void getMap()
    {

        rosbag::Bag bag;
        bag.open("/home/ras/.ros/good_map_test.bag", rosbag::bagmode::Read);

        //std::vector<std::string> topics;
        //topics.push_back(std::string("/gridmap"));

        rosbag::View view(bag, rosbag::TopicQuery("/gridmap"));

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {            
            ROS_INFO("Bag loaded");
            map_msg = m.instantiate<nav_msgs::OccupancyGrid>();
            loc_map = map_msg->data;
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


    void sensorCallback(const ras_arduino_msgs::ADConverter &sens_msg)
    {
        sensor_msg = sens_msg;
    }


    void updateLocalization()
    {        
        if ( (sensor_msg.ch1 < 30) && (sensor_msg.ch1 > 0) &&
             (sensor_msg.ch3 < 30) && (sensor_msg.ch3 > 0) &&
             (abs(sensor_msg.ch1 - sensor_msg.ch3) < 3) )
        {                        
            dist_s1 = sensor_msg.ch1/100.0;
            dist_s3 = sensor_msg.ch3/100.0;
            updateWithIR(dist_s1, dist_s3, 1); //1: left side
        }
        else if ( (sensor_msg.ch2 < 30) && (sensor_msg.ch2 > 0) &&
                  (sensor_msg.ch4 < 30) && (sensor_msg.ch4 > 0) &&
                  (abs(sensor_msg.ch2 - sensor_msg.ch4) < 3) )
        {            
            dist_s2 = sensor_msg.ch2/100.0;
            dist_s4 = sensor_msg.ch4/100.0;
            updateWithIR(dist_s2, dist_s4, 2); //2: right side
        }
        else
        {            
            //update only with odometry
            poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
            publishPose();
        }
    }

    void updateWithIR(double dist_up_sens, double dist_down_sens, int side)
    {
        dist = (dist_up_sens + dist_down_sens)/2;
        if( ((theta_prime <= eps) && (theta_prime >= 0)) ||
                ((theta_prime <= 0) && (theta_prime >= -eps)) )
        {
            //update only x_t with IR sensors            
            findWall(x_prime, y_prime, side, 1, dist);
            if(wall_x != 0)
            {                
                if(side == 1)
                {
                    x_t_ir = wall_x*resolution + (dist + robot_radius) - center_x_m;
                    ROS_INFO("side 1, x_t: %f", x_t_ir);
                }
                if(side == 2)
                {
                    x_t_ir = wall_x*resolution - (dist + robot_radius) - center_x_m;                    
                }
                ROS_INFO("update with IR: side %d, angle 0", side);
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                x_prime = x_t_ir;
                publishPose();
            }
            if(wall_x == 0)
            {
                ROS_INFO("IN IR, angle 0, no wall");
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                publishPose();
            }

        }
        else if( ((theta_prime >= M_PI - eps) && (theta_prime <= M_PI)) ||
                 ((theta_prime <= -M_PI + eps) && (theta_prime >= -M_PI)) )
        {

            //update only x_t with IR sensors            
            findWall(x_prime, y_prime, side, 3, dist);
            if(wall_x != 0)
            {
                if(side == 1)
                {
                    x_t_ir = wall_x*resolution - (dist + robot_radius) - center_x_m;
                }
                if(side == 2)
                {
                    x_t_ir = wall_x*resolution + (dist + robot_radius) - center_x_m;
                }
                ROS_INFO("update with IR: side %d, angle pi/-pi", side);
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                x_prime = x_t_ir;
                publishPose();
            }
            if(wall_x == 0)
            {
                ROS_INFO("IN IR, angle pi/-pi, no wall");
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                publishPose();
            }
        }
        else if( (theta_prime <= M_PI_2 + eps) && (theta_prime >= M_PI_2 - eps) )
        {
            //update only y_t with IR sensors
            ROS_INFO("side %d, angle pi/2", side);
            findWall(x_prime, y_prime, side, 2,dist);
            if(wall_y != 0)
            {
                if(side == 1)
                {
                    y_t_ir = wall_y*resolution + (dist + robot_radius) - center_y_m;
                }
                if(side == 2)
                {
                    y_t_ir = wall_y*resolution - (dist + robot_radius) - center_y_m;
                }
                ROS_INFO("update with IR: side %d, angle 90°", side);
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                y_prime = y_t_ir;
                publishPose();
            }
            if(wall_y == 0)
            {
                ROS_INFO("IN IR, angle 90°, no wall");
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                publishPose();
            }
        }
        else if( (theta_prime <= -M_PI_2 + eps) && (theta_prime >= -M_PI_2 - eps) )
        {
            //update only y_t with IR sensors
            ROS_INFO("side %d, angle -pi/2", side);
            findWall(x_prime, y_prime, side, 4, dist);
            if(wall_y != 0)
            {
                if(side == 1)
                {
                    y_t_ir = wall_y*resolution - (dist + robot_radius) - center_y_m;
                }
                if(side == 2)
                {
                    y_t_ir = wall_y*resolution + (dist + robot_radius) - center_y_m;
                }
                ROS_INFO("update with IR: side %d, angle -90°", side);
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                y_prime = y_t_ir;
                publishPose();
            }
            if(wall_y == 0)
            {
                ROS_INFO("IN IR, angle 90°, no wall");
                poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
                publishPose();
            }
        }
        else
        {
            ROS_INFO("IN IR, no angle condit, no wall");
            poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
            publishPose();
        }
    }


    void findWall(double x_dist, double y_dist, int side, int angle, double dist)
    {
        //cell_dist = floor(dist/resolution);
        margin_cell = 10;
        x_cell = floor((center_x_m + x_dist)/resolution);
        y_cell = floor((center_x_m + y_dist)/resolution);
        int count = 0;
        while( count <= margin_cell)
        {            
            // at angle 0
            if(side == 1 && angle == 1)
            {
                x_cell--;                
                if ( (loc_map[x_cell+width_map*y_cell] == wallValue) || (loc_map[x_cell+width_map*y_cell] == 150) )
                {
                    wall_x = x_cell;
                    wall_y = y_cell;
                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            if(side == 2 && angle == 1)
            {
                x_cell++;
                if (loc_map[x_cell+width_map*y_cell] == wallValue  || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;
                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            // at angle pi/2
            if(side == 1 && angle == 2)
            {
                y_cell--;
                if (loc_map[x_cell+width_map*y_cell] == wallValue  || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;
                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            if(side == 2 && angle == 2)
            {
                y_cell++;
                if (loc_map[x_cell+width_map*y_cell] == wallValue  || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;

                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            // at angle -pi or pi
            if(side == 1 && angle == 3)
            {
                x_cell++;
                if (loc_map[x_cell+width_map*y_cell] == wallValue || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;

                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            if(side == 2 && angle == 3)
            {
                x_cell--;
                if (loc_map[x_cell+width_map*y_cell] == wallValue || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;

                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            // at angle -pi/2
            if(side == 1 && angle == 4)
            {
                y_cell++;
                if (loc_map[x_cell+width_map*y_cell] == wallValue || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;

                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }

            }
            if(side == 2 && angle == 4)
            {
                y_cell--;
                if (loc_map[x_cell+width_map*y_cell] == wallValue || (loc_map[x_cell+width_map*y_cell] == 150) ){
                    wall_x = x_cell;
                    wall_y = y_cell;
                    ROS_INFO("wall_x: %d", wall_x);
                    ROS_INFO("wall_y: %d", wall_y);
                    return;
                }
            }
            count++;
        }        
        {
            wall_x = 0;
            wall_y = 0;
        }
    }



    void poseUpdate(double x_t, double y_t, double theta_t)
    {

        x_prime = x_prime + x_t;
        y_prime = y_prime + y_t;
        theta_prime = theta_prime + theta_t;
        theta_prime = angleBoundaries(theta_prime);
        //ROS_INFO("theta_prime %f", theta_prime);
        //ROS_INFO("x_prime %f", x_prime);
        //ROS_INFO("y_prime %f", y_prime);
        //ROS_INFO("theta_prime %f", theta_prime);                        

    }

    void publishPose()
    {
        new_pose.x = x_prime + center_x_m;
        new_pose.y = y_prime + center_y_m;
        new_pose.theta = theta_prime;

        pose_publisher.publish(new_pose);

        poseStamp_msg.pose.position.x = x_prime + center_x_m;
        poseStamp_msg.pose.position.y = y_prime + center_y_m;
        poseStamp_msg.pose.position.z = 0;

        tf::Quaternion q;
        q.setEuler(0.0, 0.0, M_PI_2 + theta_prime);
        tf::quaternionTFToMsg(q, poseStamp_msg.pose.orientation);
        poseVis_publisher.publish(poseStamp_msg);
    }

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
    loc.getInitialPose();
    //loc.getMap();


    ros::Rate loop_rate(20.0);

    while(loc.n.ok())
    {
        ros::spinOnce();
        loc.updateLocalization();
        loc.publishMap();
        loop_rate.sleep();
    }

    return 0;
}
