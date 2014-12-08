#include "ros/ros.h"
#include "math.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/ADConverter.h"
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
    ros::Subscriber encoder_subscriber;
    ros::Subscriber sensor_check_sub;
    ros::Subscriber sensor_subscriber;
    ros::Publisher map_publisher;
    ros::Publisher pose_publisher;
    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    std::vector<signed char> grid_map, loc_map;
    geometry_msgs::PoseStamped poseStamp_msg;
    robot_msgs::IrTransformMsg sensCheck_msg;
    ras_arduino_msgs::ADConverter sensor_msg;

    double x_t_ir, y_t_ir, theta_t_ir;
    double x_t_odom, y_t_odom, theta_t_odom;
    double x_prime, y_prime, theta_prime;
    double resolution;
    int center_x, center_y, height_robot, width_robot, width_map, cellNumber;
    int x_pose_cell_map, y_pose_cell_map;
    int wall_x, wall_y;
    int x_t_cell, y_t_cell;
    double center_x_m, center_y_m, dist;
    double x_pose_cell, y_pose_cell, prev_x_pose_cell, prev_y_pose_cell;
    double dist_s1, dist_s2, dist_s3, dist_s4;
    double b, r, sampleTime, robot_radius;
    double eps;


    Localize()
    {
        n = ros::NodeHandle();
    }

    ~Localize()
    {}
    void init()
    {
        sensor_subscriber = n.subscribe("/ir_sensor_cm", 1, &Localize::sensorCallback, this);
        sensor_check_sub = n.subscribe("/transformed_ir_points",1, &Localize::sensorCallback,this);
        encoder_subscriber = n.subscribe("/arduino/encoders", 1, &Localize::encoderCallback,this);        
        map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/loc/savedmap",1);
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/loc/pose", 1);

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

        sampleTime = 0.05;
        eps = 8*(M_PI/180);//equivalent to 8 deg; given in [rad]
        ROS_INFO_ONCE("EPSILON", eps);
    }   

    void getInitialPose()
    {
        x_prime = 0;
        y_prime = 0;
        if ((sensCheck_msg.s1 == true) && (sensCheck_msg.s3 == true))
            theta_prime = asin((sensor_msg.ch1 - sensor_msg.ch3)/13.5);
        else if ((sensCheck_msg.s2 == true) && (sensCheck_msg.s4 == true))
            theta_prime = asin((sensor_msg.ch4 - sensor_msg.ch2)/13.5);
        else
            theta_prime = 0;
    }
    void getMap()
    {

        rosbag::Bag bag;
        bag.open("/home/ras/.ros/map_test_2.bag", rosbag::bagmode::Read);

        rosbag::View view(bag, rosbag::TopicQuery("/gridmap"));

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            ROS_INFO("Bag successfully loaded... probably");
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

        updateLocalization();

    }


    void sensorCallback(const ras_arduino_msgs::ADConverter &sens_msg)
    {
        sensor_msg = sens_msg;
    }

    void sensorCheckCallback(const robot_msgs::IrTransformMsg &check_msg)
    {
        sensCheck_msg = check_msg;
    }

    void updateLocalization()
    {
        if ((sensCheck_msg.s1 == true) && (sensCheck_msg.s3 == true))
        {
            dist_s1 = sensor_msg.ch1/100.0;
            dist_s3 = sensor_msg.ch3/100.0;
            updateWithIR(dist_s1, dist_s3, 1); //1: left side
        }
        else if ((sensCheck_msg.s2 == true) && (sensCheck_msg.s4 == true))
        {
            dist_s2 = sensor_msg.ch2/100.0;
            dist_s4 = sensor_msg.ch4/100.0;
            updateWithIR(dist_s2, dist_s4, 2); //2: left side
        }
        else
        {
            //update only with odometry
            poseUpdate(x_t_odom, y_t_odom, theta_t_odom);
        }
    }

    void updateWithIR(double dist_up_sens, double dist_down_sens, int side)
    {
        dist = (dist_up_sens + dist_down_sens)/2;
        if( (theta_prime == eps) || (theta_prime == -eps) )
        {
            //update only x_t with IR sensors
            ROS_INFO_ONCE("side %d, angle 0", side);
            findWall(x_prime, y_prime, side, 1);
            if(side == 1)
            {
                x_t_ir = wall_x*resolution + (dist + robot_radius) - center_x_m;
            }
            if(side == 2)
            {
                x_t_ir = wall_x*resolution - (dist + robot_radius) - center_x_m;
            }
            poseUpdate(x_t_ir, y_t_odom, theta_t_odom);
        }
        else if((theta_prime == M_PI - eps) || (theta_prime == -M_PI + eps))
        {
            //update only x_t with IR sensors
            ROS_INFO_ONCE("side %d, angle pi/-pi", side);
            findWall(x_prime, y_prime, side, 3);
            if(side == 1)
            {
                x_t_ir = wall_x*resolution - (dist + robot_radius) - center_x_m;
            }
            if(side == 2)
            {
                x_t_ir = wall_x*resolution + (dist + robot_radius) - center_x_m;
            }
            poseUpdate(x_t_ir, y_t_odom, theta_t_odom);
        }
        else if( (theta_prime == M_PI_2 + eps) || (theta_prime == M_PI_2 - eps) )
        {
            //update only y_t with IR sensors
            ROS_INFO_ONCE("side %d, angle pi/2", side);
            findWall(x_prime, y_prime, side, 2);
            if(side == 1)
            {
                y_t_ir = wall_y*resolution + (dist + robot_radius) - center_y_m;
            }
            if(side == 2)
            {
                y_t_ir = wall_y*resolution - (dist + robot_radius) - center_y_m;
            }
            poseUpdate(x_t_odom, y_t_ir, theta_t_odom);
        }
        else if( (theta_prime == -M_PI_2 + eps) || (theta_prime == -M_PI_2 - eps) )
        {
            //update only y_t with IR sensors
            ROS_INFO_ONCE("side %d, angle -pi/2", side);
            findWall(x_prime, y_prime, side, 4);
            if(side == 1)
            {
                y_t_ir = wall_y*resolution - (dist + robot_radius) - center_y_m;
            }
            if(side == 2)
            {
                y_t_ir = wall_y*resolution + (dist + robot_radius) - center_y_m;
            }
            poseUpdate(x_t_odom, y_t_ir, theta_t_odom);
        }
    }


    void findWall(double x_dist, double y_dist, int side, int angle)
    {
        int x_cell = floor((center_x_m + x_dist)/resolution);
        int y_cell = floor((center_x_m + y_dist)/resolution);
        while(loc_map[x_cell+width_map*y_cell] != 150)
        {
            // at angle 0
            if(side == 1 && angle == 1)
            {
                x_cell--;
            }
            if(side == 2 && angle == 1)
            {
                x_cell++;
            }
            // at angle pi/2
            if(side == 1 && angle == 2)
            {
                y_cell--;
            }
            if(side == 2 && angle == 2)
            {
                y_cell++;
            }
            // at angle -pi or pi
            if(side == 1 && angle == 3)
            {
                x_cell++;
            }
            if(side == 2 && angle == 3)
            {
                x_cell--;
            }
            // at angle -pi/2
            if(side == 1 && angle == 4)
            {
                y_cell++;
            }
            if(side == 2 && angle == 4)
            {
                y_cell--;
            }
        }
        wall_x = x_cell;
        wall_y = y_cell;

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

        poseStamp_msg.pose.position.x = x_prime + center_x_m;
        poseStamp_msg.pose.position.y = y_prime + center_y_m;
        poseStamp_msg.pose.position.z = 0;

        tf::Quaternion q;
        q.setEuler(0.0, 0.0, M_PI_2 + theta_prime);
        tf::quaternionTFToMsg(q, poseStamp_msg.pose.orientation);
        pose_publisher.publish(poseStamp_msg);

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
    loc.getMap();


    ros::Rate loop_rate(20.0);

    while(loc.n.ok())
    {
        ros::spinOnce();
        loc.publishMap();
        loop_rate.sleep();
    }

    return 0;
}
