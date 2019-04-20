#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include "Mini_SLAM/WriteToFile.h"
#include <fstream>
#include<mutex>
#include <queue>


using namespace std;
class mini_SLAM
{
private:

	int world_map[10][10];
	int robot_x;
	int robot_y;
	std::pair<int, int> robot_orientation;

	std::queue<std::pair<int, int**> > maps; //we constrcut this for accessing maps prior to update for correct loop closure detection 

	int current_sensor_reading[3][5];
	int previous_sensor_reading[3][5]; // as local map for scan matcher

	bool first_map_fill; 

	ros::NodeHandle nh;	
	ros::Subscriber pointcloud_sub;

	ros::ServiceServer service;

	std::mutex map_pose_update;
	int sensor_data_id;

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

		service = nh.advertiseService("write_to_file", &mini_SLAM::write_to_file, this);
  
  		ROS_INFO("Map Writing Server up and running.");

  		sensor_data_id = -1;
	}
    
    void run();
    void detect_loop();
    void bundle_adjustment();
    void update_robot_pose();
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void scan_matcher();
    void turn_local_map(int**);
    bool write_to_file(Mini_SLAM::WriteToFile::Request&, Mini_SLAM::WriteToFile::Response&);
    void update_occupancy_grid();
   
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
	ros::spin();
	thread1->join();

}
void mini_SLAM::detect_loop()
{
	// for detecting loops we search within a local map of 5*5 around current estimated robot pose in all orientations
	

	int** local_map = new int*[5];
	for (int i = 0 ; i < 5 ; i++)
		local_map[i] = new int[5];

	int current_sensor_index = 0;

	ros::Rate loop_rate(1);
	while (ros::ok())
	{

		if (sensor_data_id > 0) //we don't want to process one scan twice or process the first scan for loop closure
		{
			for (int i = 0; i < 5 ; i++)
			{
				for (int j = 0; j < 5 ; j++)
					local_map[i][j] = -1;
			}
			local_map[2][2] = 0; //robot_pose
		   	
		   	int sensor_to_check[3][5];
		   	
		   	{
		   		unique_lock<mutex> lock(map_pose_update);

		   		for (int i = 0; i < 3 ; i++)
		   		{
		   			for (int j = 0; j < 5; j++)
		   				sensor_to_check[i][j] = current_sensor_reading[i][j];
		   		}

		   		current_sensor_index = sensor_data_id;
		   		int** global_map_to_use = NULL;

	   			while (maps.front().first < current_sensor_index)
	   			{
	   				if (global_map_to_use)
	   				{
	   					for (int i = 0 ; i < 10; i++)
	   						delete global_map_to_use[i];
	   					delete global_map_to_use;
	   				}
	   				global_map_to_use = maps.front().second;
	   				
	   				if (maps.front().first != current_sensor_index - 1) //we don't want to encounter an empty map, so we keep the last map before current sensor index
	   					maps.pop();
	   				else 
	   					break;
	   			}
		   		

			   	int localmap_min_x = std::max(robot_x - 2, 0);
				int localmap_min_y = std::max(robot_y - 2, 0);
				int localmap_max_x = std::min(robot_x + 2, 9);
				int localmap_max_y = std::min(robot_y + 2, 9);

				for (int i = localmap_min_x ; i <= localmap_max_x ; i++)
				{
					for (int j = localmap_min_y; j <= localmap_max_y; j++)
					{
						local_map[i - localmap_min_x][j - localmap_min_y] = global_map_to_use[i][j];
					}
				}
			}

			int num_rotations = 0; //since we wanna check 90, -90 and 180 degree rotations of local map as well
			int max_similarity_score = 0;
			while (num_rotations < 4)
			{
				for (int translation = 0; translation < 3; translation++) //in 5*5 local map, there are three placements for a 3*5 sensor reading, we check all
				{
					int current_score = 0;
					for (int i = translation+0; i < translation+3; i++)
					{
						for (int j = 0; j < 5; j++)
						{
							if (local_map[i][j] != -1 && sensor_to_check[i-translation][j] != -1)
							{
								(local_map[i][j] == sensor_to_check[i-translation][j])?current_score++:current_score--;
							}
						}
					}
					if (current_score > max_similarity_score)
					{
						max_similarity_score = current_score;
					}
				}
				turn_local_map(local_map);
				num_rotations++;
			}

			if (max_similarity_score >= 14)
				std::cout << "loop closure detected" << std::endl;
			
			
		}
		loop_rate.sleep();

	}

	for (int i = 0 ; i < 5 ; i++)
		delete local_map[i];
	delete local_map;


	

/*	std::cout << "================== local map =============" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			std::cout << local_map[i][j] << " ";
		}
		std::cout << std::endl;
	}*/

/*	if (robot_orientation.second == -1) //we should turn the local map 90 degree clock wise for simpler scan matching
	{
		turn_local_map(local_map);
	}
	if (robot_orientation.first == 1) //we should turn the local map 180 degree clock wise
	{
		for (int i=0; i < 2 ; i++)
			turn_local_map(local_map);
	}
	if (robot_orientation.second == 1) //we should turn the local map 270 degree clock wise
	{
		for (int i=0; i < 3 ; i++)
			turn_local_map(local_map);
	}
		*/
//	std::thread* thread2 = new std::thread(&mini_SLAM::bundle_adjustment, this);
//	thread2->join();

}
void mini_SLAM::bundle_adjustment()
{}
void mini_SLAM::scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(*msg, pcl_cloud);

	int robot_pos_within_sensor_x = 2;
	int robot_pos_within_sensor_y = 2;

	{
		unique_lock<mutex> lock(map_pose_update);
		for (int i = 0; i <= 2; i++) //here we reset the matrix
		{
			for (int j = 0; j <= 4; j++)
			{
				current_sensor_reading[i][j] = -1;
			}
		}

		
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
		sensor_data_id++;
	}
/*	std::cout << "=================current reading =============" << std::endl;
	for (int i = 0 ; i < 3; i++)
	{
		for (int j = 0 ; j < 5; j++)
		{
			std::cout << current_sensor_reading[i][j] << " ";
		}
		std::cout << std::endl;
	}*/

	if (!first_map_fill)
	{
		scan_matcher();
		update_occupancy_grid();
	}

	if (first_map_fill)
	{
		first_map_fill = false;

		{
			unique_lock<mutex> lock(map_pose_update);
			int world_start_x = robot_x - 2;
			int world_start_y = robot_y - 2;

			for(int i = world_start_x ; i <= world_start_x + 2; i++)
			{
				for(int j = world_start_y ; j <= world_start_y + 4; j++)
					world_map[i][j] = current_sensor_reading[i - world_start_x][j - world_start_y];
			}
		}

		int** map_copy = new int*[10];
		for (int i = 0 ; i < 10; i++)
		{
			map_copy[i] = new int[10];
			for (int j = 0 ; j < 10 ; j++)
			{
				map_copy[i][j] = world_map[i][j];
			}
		}

		maps.push(std::make_pair(sensor_data_id, map_copy));
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0 ; j < 5 ; j++)
		{
			previous_sensor_reading[i][j] = current_sensor_reading[i][j];
		}
	}
}
void mini_SLAM::scan_matcher()
{
	// for scan matching we only check one cell forward, and 90 degree rotation left and 90 degree rotation right
	// for local map we only assume the previous scan

	//check moving forward: in this case, the top two rows of previous_sensor_reading should be compared with the bottom two rows of current sensor reading
	int match_forward = 0;
	for (int i = 1 ; i < 3; i++)
	{
		for (int j = 0 ; j < 5; j++)
		{
			if (current_sensor_reading[i][j] != -1 && previous_sensor_reading[i-1][j] != -1)
			{
				(current_sensor_reading[i][j] - previous_sensor_reading[i-1][j]) == 0 ? match_forward++ : match_forward--;
			}
		}
	}

	// turn left 90 degree, the left 3*3 of previous sensor reading compared to right 3*3 of current sensor reading

	int match_turn_left = 0;
	for (int i = 0 ; i < 3; i++)
	{
		for (int j = 2 ; j < 5; j++)
		{
			if (current_sensor_reading[i][j] != -1 && previous_sensor_reading[4-j][i] != -1)
			{
				(current_sensor_reading[i][j] - previous_sensor_reading[4-j][i]) == 0 ? match_turn_left++ : match_turn_left--;
			}
		}
	}

	// turn right 90 degree, the right 3*3 of previous sensor reading compared to left 3*3 of current sensor reading

	int match_turn_right = 0;
	for (int i = 0 ; i < 3; i++)
	{
		for (int j = 0 ; j < 3; j++)
		{
			if (current_sensor_reading[i][j] != -1 && previous_sensor_reading[j][4-i] != -1)
			{
				(current_sensor_reading[i][j] - previous_sensor_reading[j][4-i]) == 0 ? match_turn_right++ : match_turn_right--;
			}
		}
	}

	//here based on the calculated matches, we update the robot pose (Localization)

	{
		unique_lock<mutex> lock(map_pose_update);
		if (match_forward > match_turn_right && match_forward > match_turn_left)
		{
			std::cout << "move forward" << std::endl;
			robot_x = robot_x + robot_orientation.first;
			robot_y = robot_y + robot_orientation.second;
		}
		else if (match_turn_left > match_turn_right && match_turn_left > match_forward)
		{
			std::cout << "turn left" << std::endl;
			if (robot_orientation.first != 0)
			{
				robot_orientation.second = robot_orientation.first;
				robot_orientation.first = 0;
			}
			else
			{
				robot_orientation.first = -robot_orientation.second;
				robot_orientation.second = 0;
			}
		}
		else if (match_turn_right > match_forward && match_turn_right > match_turn_left)
		{
			std::cout << "turn right" << std::endl;
			if (robot_orientation.first != 0)
			{
				robot_orientation.second = -robot_orientation.first;
				robot_orientation.first = 0;

			}
			else
			{
				robot_orientation.first = robot_orientation.second;
				robot_orientation.second = 0;

			}
		}
	}

}

void mini_SLAM::turn_local_map(int** local_map)
{
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j <= 2; j++)
		{
			int temp = local_map[i][j];
			local_map[i][j] = local_map[4-j][i];
			local_map[4-j][i] = local_map[4-i][4-j];
			local_map[4-i][4-j] = local_map[j][4-i];
			local_map[j][4-i] = temp;
		}
	}
/*	std::cout << "================== turned local map =============" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			std::cout << local_map[i][j] << " ";
		}
		std::cout << std::endl;
	}*/
}

bool mini_SLAM::write_to_file(Mini_SLAM::WriteToFile::Request  &req, Mini_SLAM::WriteToFile::Response  &response)
{
	std::ofstream myfile;
    myfile.open ("global_map.txt");

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			myfile << world_map[i][j] << ",";
		}
		myfile << std::endl;
	}

	myfile.close();
	std::cout << "Wrote global map to global_map.txt" << std::endl;
}

void mini_SLAM::update_occupancy_grid()
{
	unique_lock<mutex> lock(map_pose_update);
	//here we update the map (Mapping)
	if (robot_orientation.first == -1)
	{
		int world_start_x = robot_x - 2;
		int world_start_y = robot_y - 2;

		for(int i = 0 ; i <= 2; i++)
		{
			for(int j = 0 ; j <= 4; j++)
			{
				if (current_sensor_reading[i][j] != -1)
					world_map[world_start_x+i][world_start_y+j] = current_sensor_reading[i][j];
			}
		}
	}
	else if (robot_orientation.first == 1)
	{
		int world_start_x = robot_x;
		int world_start_y = robot_y - 2;

		for(int i = 0 ; i <= 2; i++)
		{
			for(int j = 0 ; j <= 4; j++)
			{
				if (current_sensor_reading[i][j] != -1)
					world_map[world_start_x+(2-i)][world_start_y+(4-j)] = current_sensor_reading[i][j];
			}
		}
	}
	else if (robot_orientation.second == -1)
	{
		int world_start_x = robot_x - 2;
		int world_start_y = robot_y - 2;

		for(int i = 0 ; i <= 2; i++)
		{
			for(int j = 0 ; j <= 4; j++)
			{
				if (current_sensor_reading[i][j] != -1)
					world_map[world_start_x+(4-j)][world_start_y+i] = current_sensor_reading[i][j];
			}
		}
	}
	else 
	{
		int world_start_x = robot_x - 2;
		int world_start_y = robot_y;

		for(int i = 0 ; i <= 2; i++)
		{
			for(int j = 0 ; j <= 4; j++)
			{
				if (current_sensor_reading[i][j] != -1)
					world_map[world_start_x+j][world_start_y+(2-i)] = current_sensor_reading[i][j];
			}
		}
	}

	int** map_copy = new int*[10];
	for (int i = 0 ; i < 10; i++)
	{
		map_copy[i] = new int[10];
		for (int j = 0 ; j < 10 ; j++)
		{
			map_copy[i][j] = world_map[i][j];
		}
	}

	maps.push(std::make_pair(sensor_data_id, map_copy)); 


}
