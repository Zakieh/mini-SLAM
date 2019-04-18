#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"


class robot_world
{
private:

	int world_map[10][10];
	int robot_x;
	int robot_y;
	int robot_orientation; 

	ros::NodeHandle nh;	

public:

	robot_world()
	{
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (i > 0 && i < 9 && j > 0 && j < 9)
					world_map[i][j] = 0;
				else
					world_map[i][j] = 1;
			}
		}
		world_map[0][6] = 0;
		world_map[3][3] = 1;
		world_map[3][6] = 1;
		world_map[4][0] = 0;
		world_map[9][0] = 0;
		world_map[5][0] = 0;
		world_map[6][3] = 1;
		world_map[6][6] = 1;
		world_map[9][7] = 0;
		world_map[9][8] = 0;

		//initializing robot first pose
		robot_x = 9;
		robot_y = 7;
		robot_orientation = 0;
	}
    
    void update_robot_pose();

   
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_node");
	
	robot_world my_world;
}