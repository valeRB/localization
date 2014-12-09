#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "visualization_msgs/Marker.h"
#include "robot_msgs/detectedObject.h"


class GoalFinder
{
public:
    ros::NodeHandle n;
    ros::Subscriber costmap_subscriber;
    ros::Subscriber object_subscriber;
    std::vector<robot_msgs::detectedObject> objects;
    nav_msgs::OccupancyGrid costmap;
    int obj_cell_x, obj_cell_y, max_cells;

    GoalFinder()
    {
        n = ros::NodeHandle();
    }

    ~GoalFinder()
    {}
    void init()
    {
        costmap_subscriber = n.subscribe("/costmap", 1, &GoalFinder::getMapCallback, this);
        object_subscriber = n.subscribe("/map/object", 1, &GoalFinder::getObjects, this);
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

    void getMapCallback(const nav_msgs::OccupancyGrid &map_msg)
    {
        costmap = map_msg;
    }

    void getObjects(const std::vector<robot_msgs::detectedObject> &obj_msg)
    {
        objects = obj_msg;
    }

    void findGoalForPath()
    {
        for(int i = obj_cell_x - max_cells; i <= obj_cell_x + max_cells; i++)
        {
            for(int j = obj_cell_y - max_cells; j <= obj_cell_y + max_cells; i++)
            {

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
