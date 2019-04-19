#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>


class mini_SLAM
{
private:

	int world_map[10][10];
	int robot_x;
	int robot_y;
	std::pair<int, int> robot_orientation;

	bool first_map_fill; 

	ros::NodeHandle nh;	
	ros::Subscriber pointcloud_sub;

public:

	mini_SLAM()
	{
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				world_map[i][j] = -1;
			}
		}
		
		//initializing robot first pose
		robot_x = 9; //I assume to know the initial robot position
		robot_y = 7;
		robot_orientation.first = -1; //regarding our map coordinate frame which (0,0) is at the top left
		robot_orientation.second = 0;

		first_map_fill = true;

		//initializing the subscriber
		pointcloud_sub = nh.subscribe("cloud", 100, &mini_SLAM::scanCallback, this);
	}
    
    void run();
    void detect_loop();
    void bundle_adjustment();
    void update_robot_pose();
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
   
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "second_node");
	
	mini_SLAM mySLAM;
	mySLAM.run();
}

void mini_SLAM::run()
{
	std::thread* thread1 = new std::thread(&mini_SLAM::detect_loop, this);
	std::thread* thread2 = new std::thread(&mini_SLAM::bundle_adjustment, this);
	ros::spin();

}
void mini_SLAM::detect_loop()
{}
void mini_SLAM::bundle_adjustment()
{}
void mini_SLAM::scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(*msg, pcl_cloud);

	int current_sensor_reading[3][5] = {{-1, -1, -1, -1, -1},
										{-1, -1, -1, -1, -1},
										{-1, -1, 0, -1, -1}}; //the zero is robot index which is (2, 2)

	int robot_pos_within_sensor_x = 2;
	int robot_pos_within_sensor_y = 2;


	for(int i = 0 ; i < pcl_cloud.points.size(); i++) //here we fill out the ones
	{
		int obstacle_x = robot_pos_within_sensor_x - pcl_cloud.points[i].x;
		int obstacle_y = robot_pos_within_sensor_y - pcl_cloud.points[i].y;
		current_sensor_reading[obstacle_x][obstacle_y] = 1;
	}

	for (int i = 0; i <= 2; i++) //here we fill out the zeros
	{
		for (int j = 0; j <= 4; j++)
		{
				
			if (current_sensor_reading[i][j] != 1)
			{
				bool is_observable = false;
				if (abs(robot_pos_within_sensor_x - i) < 2 && abs(robot_pos_within_sensor_y - j) < 2) //this shows we are in the closer square, don't need to worry about occlusion
				{
					is_observable = true;
				}
				else //Here we should check if the cell in front is occupied or not
				{

					int current_x_check = i, current_y_check = j;
					if (abs(robot_pos_within_sensor_x - i)>1)
						current_x_check = (robot_pos_within_sensor_x > i)?robot_pos_within_sensor_x-1:robot_pos_within_sensor_x+1;
					if (abs(robot_pos_within_sensor_y - j)>1)
						current_y_check = (robot_pos_within_sensor_y > j)?robot_pos_within_sensor_y-1:robot_pos_within_sensor_y+1;

					if (current_sensor_reading[current_x_check][current_y_check] != 1)
					{
						is_observable = true;
					}
				}

				if (is_observable)
				{
					current_sensor_reading[i][j] = 0; 
				}
			} 
		}
	}
	std::cout << "==============================" << std::endl;
	for (int i = 0 ; i < 3; i++)
	{
		for (int j = 0 ; j < 5; j++)
		{
			std::cout << current_sensor_reading[i][j] << " ";
		}
		std::cout << std::endl;
	}

/*	if (first_map_fill)
	{
		first_map_fill = false;

		int obstacle_x, obstacle_y;
		for(int i = 0 ; i < pcl_cloud.points.size(); i++)
		{
			obstacle_x = (robot_orientation.first * pcl_cloud.points[i]) + robot_x;
			obstacle_y = (robot_orientation.first * pcl_cloud.points[i]) + robot_x;
			world_map[obstacle_x][obstacle_y] = 1;
		}
	}*/

	//std::cout << "pcl size " << pcl_cloud.points.size() << std::endl;

}
