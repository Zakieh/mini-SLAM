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
		robot_x = 9;
		robot_y = 7;
		robot_orientation.first = -1; //regarding our map coordinate frame which (0,0) is at the top left
		robot_orientation.second = 0; 

		//initializing the publisher
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
{}
