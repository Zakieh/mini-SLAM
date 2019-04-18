#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>




class robot_world
{
private:

	int world_map[10][10];
	int robot_x;
	int robot_y;
	std::pair<int, int> robot_orientation; 

	ros::NodeHandle nh;	
	ros::Publisher pointcloud_pub;

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
		robot_orientation.first = -1; //regarding our map coordinate frame which (0,0) is at the top left
		robot_orientation.second = 0; 

		//initializing the publisher
		pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1000);
	}
    
    void run();
    void update_robot_pose();
    void create_publish_pointcloud(int, int, int, int);
   
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_node");
	
	robot_world my_world;
	my_world.run();
}

void robot_world::run()
{

	ros::Rate loop_rate(1);

	while (ros::ok())
    {
    	int min_cover_x = 10, max_cover_x = 0, min_cover_y = 10, max_cover_y = 0;
	    
	    //at first step, we should find the sorrounding square using the current robot pose
	    //We use vector dot multiplication for finding cells from -90 to 90 degree regarding current robot orientation

    	for (int i = -2; i <= 2; i++)
    	{
    		for (int j = -2; j <= 2; j++)
    		{
    			if (robot_x+i >= 0 && robot_x+i < 10 && robot_y+j >=0 && robot_y+j < 10) // here we check boundaries
    			{
    				if ((i*robot_orientation.first)+(j*robot_orientation.second) >= 0) //here the dot multiplication happens
    				{
    					if (robot_x+i < min_cover_x)
    						min_cover_x = robot_x + i;
    					if (robot_x+i > max_cover_x)
    						max_cover_x = robot_x + i;

    					if (robot_y+j < min_cover_y)
    						min_cover_y = robot_y + j;
    					if (robot_y+j > max_cover_y)
    						max_cover_y = robot_y + j;
    				}
    			}
    		}
    	}

    	//std::cout << min_cover_x << " " << min_cover_y << " " << max_cover_x << " " << max_cover_y << std::endl;

    	// now we construct the point cloud using the boundaries
		create_publish_pointcloud(min_cover_x, max_cover_x, min_cover_y, max_cover_y);
		loop_rate.sleep();
	    
    }

}
void robot_world::create_publish_pointcloud(int min_x, int max_x, int min_y, int max_y)
{
	sensor_msgs::PointCloud2 current_pointcloud;

	//instead of filling out the sensor_msg manually, I create a pcl point cloud and convert it to PointCloud2 later on for simplicity
	pcl::PointCloud<pcl::PointXYZ> cloud;

	int count_obstacles = 0;
	for (int i = min_x; i <= max_x; i++)
	{
		for (int j = min_y; j <= max_y; j++)
		{
			//std::cout << "borders " << i << " " << j << std::endl;

				
			if (world_map[i][j] == 1)
			{
				//std::cout << "1 is here " << i << " " << j << std::endl;
				bool is_observable = false;
				if (abs(robot_x-i) < 2 && abs(robot_y-j) < 2) //this shows we are in the closer square, don't need to worry about occlusion
				{
					is_observable = true;
				}
				else 
				{

					int current_x_check = i, current_y_check = j;
					if (abs(robot_x-i)>1)
						current_x_check = (robot_x > i)?robot_x-1:robot_x+1;
					if (abs(robot_y-j)>1)
						current_y_check = (robot_y > j)?robot_y-1:robot_y+1;

					if (world_map[current_x_check][current_y_check] != 1)
					{
						is_observable = true;
					}
				}

				if (is_observable)
				{
					pcl::PointXYZ obstacle;
					obstacle.x = robot_x-i;
					obstacle.y = robot_y-j;
					cloud.push_back(obstacle);
					count_obstacles++;
					std::cout << "obstacle " << obstacle.x << " " << obstacle.y << std::endl; 
				}
			} 
		}
	}

}