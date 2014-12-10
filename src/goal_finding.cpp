#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "visualization_msgs/Marker.h"
#include "robot_msgs/detectedObject.h"
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"

class GoalFinder
{
public:
    ros::NodeHandle n;
    ros::Subscriber costmap_subscriber;
    ros::Subscriber object_subscriber;
    std::vector<robot_msgs::detectedObject> objects;
    robot_msgs::detectedObject::ConstPtr obj;
    nav_msgs::OccupancyGrid costmap;
    int pos_cell_x, pos_cell_y, max_cells;
    int resolution, width_map;

    GoalFinder()
    {
        n = ros::NodeHandle();
    }

    ~GoalFinder()
    {}
    void init()
    {
        costmap_subscriber = n.subscribe("/costmap", 1, &GoalFinder::getMapCallback, this);
        object_subscriber = n.subscribe("/allObjects", 1, &GoalFinder::getObjects, this);
        //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/loc/pose", 1);

        cellNumber = 500*500;
        resolution = 0.02; //[m]
        center_x = 250; //[cell]
        center_y = 250;
        center_x_m = 5.0; //[m]
        center_y_m = 5.0; //[m]
        height_robot = 10;
        width_robot = 10;
        width_map = 500;

    }

    void getMap()
    {

        rosbag::Bag bag;
        bag.open("/home/ras/.ros/map_test_2.bag", rosbag::bagmode::Read);

        //topics.push_back(std::string("/gridmap"));

        rosbag::View obj_view(bag, rosbag::TopicQuery("/allObjects"));

        for (rosbag::View::iterator m = obj_view.begin(); m != obj_view.end(); m++)
        {

            obj = (*m).instantiate<robot_msgs::detectedObject>();
            objects.push_back(obj);
        }
        ROS_INFO("Received %d objects", objects.size());
        bag.close();
    }

    void getMapCallback(const nav_msgs::OccupancyGrid &map_msg)
    {
        costmap = map_msg;
    }

    void findGoalForPath()
    {
        for (std::vector<robot_msgs::detectedObject>::iterator it = detected_objects.begin(); it != detected_objects.end(); ++it)
        {
            robot_msgs::detectedObject Obj = *it;
            pos_cell_x = floor(Obj.position.x/resolution);
            pos_cell_y = floor(Obj.position.y/resolution);
            if(costmap.data[pos_cell_x + width_map*pos_cell_y] != 0)
            {
                count = 1;
                while(count <= 10)
                if(costmap.data[(pos_cell_x+count) + width_map*(pos_cell_y)] == 0)
                {
                    obj_cell_x = pos_cell_x + count;
                    obj_cell_y = pos_cell_y;
                    break;
                }
                else if(costmap.data[(pos_cell_x+count) + width_map*(pos_cell_y+count)] == 0)
                {
                    obj_cell_x = pos_cell_x + count;
                    obj_cell_y = pos_cell_y + count;
                    break;
                }
                else if(costmap.data[(pos_cell_x) + width_map*(pos_cell_y+count)] == 0)
                {
                    obj_cell_x = pos_cell_x;
                    obj_cell_y = pos_cell_y + count;
                    break;
                }
                else if(costmap.data[(pos_cell_x-count) + width_map*(pos_cell_y+count)] == 0)
                {
                    obj_cell_x = pos_cell_x - count;
                    obj_cell_y = pos_cell_y + count;
                    break;
                }
                //...
                else
                {
                    count++;
                }



            }

        }
    }

private:


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");

    GoalFinder find;

    ros::Rate loop_rate(20.0);

    while(find.n.ok())
    {
        ros::spinOnce();
        //loc.updateLocalization();
        //loc.publishMap();
        loop_rate.sleep();
    }

    return 0;
}
