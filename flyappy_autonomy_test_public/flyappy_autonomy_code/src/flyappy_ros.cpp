#include "flyappy_autonomy_code/flyappy_ros.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <numeric>

constexpr uint32_t QUEUE_SIZE = 5u;

const int OG_SIZE_X = 404;
const int OG_SIZE_Y = 355;
const int RESOLUTION = 100;
const int DILATATION_SIZE = 49;

double delta_v_x = 0;
double position_y = 149;
double target = 202, slow_x = 0;
std::vector<double> target_vector = {};

ros::Time current_time;
cv::Mat_<uint8_t> laser_map = cv::Mat_<uint8_t>(OG_SIZE_X, OG_SIZE_Y); 
cv::Mat_<uint8_t> occupancy_grid = cv::Mat_<uint8_t>(OG_SIZE_X, OG_SIZE_Y, 0xFF); 

FlyappyRos::FlyappyRos(ros::NodeHandle& nh):
        pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
        sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,this)),
        sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,&FlyappyRos::laserScanCallback, this)),
        sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,&FlyappyRos::gameEndedCallback, this)){
}


void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ros::Duration time_span = ros::Time::now() - current_time;
    current_time = ros::Time::now();

    delta_v_x = (time_span.toSec() * msg->x) * RESOLUTION;
    position_y += (time_span.toSec() * msg->y) * RESOLUTION;

    geometry_msgs::Vector3 acc_cmd;

    //Add a slow factor if the bird is too close from an obstacle
    slow_x = 0;
    for (int j = position_y-40; j<=position_y+40; j++) {
        for (int i = 10; i < 50; i++) {
            if (abs(position_y-target)>5 && occupancy_grid(j, i) != 255 && hypot(j-position_y, i) < 30) {
                slow_x = -6;
            }
        }
    }

    if (abs(target-position_y)>30) {
        slow_x = -6;
    }

    if ((msg->x < 3 && abs(target-position_y)<10) || msg->x < 0.5) {
        acc_cmd.x = 2.5;
    }
    else if (msg->x >= 4.2) {
        acc_cmd.x = -0.5;
    }
    else { 
        acc_cmd.x = 1.5 + slow_x;
    }
    std::cout << msg->x << std::endl;

    acc_cmd.y = (target - position_y)*4 - msg->y*50;

    pub_acc_cmd_.publish(acc_cmd);

}


void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //Draw on a map the points detected by the laser
    laser_map.setTo(cv::Scalar(255));

    for (int i = 0; i<msg->ranges.size(); i ++) {
        if (msg->intensities[i]) {
            int x = (int) (msg->ranges[i] * cos(msg->angle_min + i*msg->angle_increment) * RESOLUTION);
            int y = 149-position_y + 256 - (int) (msg->ranges[i] * sin(msg->angle_min + i*msg->angle_increment) * RESOLUTION);
            
            laser_map(y, x) = 0;
        }
    }

    erode(laser_map, laser_map, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATATION_SIZE, DILATATION_SIZE), cv::Point(10,10)));


    //Compute the occupancy grid from the laser map, and shift it according to the travelled distance
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, -delta_v_x, 0, 1, 0);
    cv::warpAffine(occupancy_grid, occupancy_grid, trans_mat, occupancy_grid.size());

    for (int j=0; j<OG_SIZE_X; j++) {
        for (int i=0; i<OG_SIZE_Y; i++) {
            if (i>=(OG_SIZE_Y-ceil(delta_v_x))) {
                occupancy_grid(j, i) = 255;
            }
            if (!laser_map(j, i)) {
                occupancy_grid(j, i) = laser_map(j, i);
            }
        }
    }

    //Compute the targeted y position as the closest position from the center of the occupancy grid which is not occupied for a certain range on x
    target_vector = {};
    for (int j = 100; j<OG_SIZE_X/2; j++) {
        bool free_row1 = true, free_row2 = true;
        for (int i = 15; i < 145; i++) {
            if (occupancy_grid(j, i) != 255) {
                free_row1 = false;
            }
            if (405-j<0 || occupancy_grid(OG_SIZE_X-j, i) != 255) {
                free_row2 = false;
            }
            if (!free_row1 && !free_row2) {
                break;
            }
        }
        if (free_row1) {
            target_vector.push_back(OG_SIZE_X-j);
        }
        if (free_row2) {
            target_vector.push_back(j);
        }
    }

    if (!target_vector.empty()) {
        target = std::accumulate(target_vector.begin(), target_vector.end(),0)/target_vector.size();
    }

    cv::imshow("Laser Map", laser_map);
    cv::imshow("Occupancy Grid", occupancy_grid);
    cv::waitKey(1);
}


void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{    
    //Reset of the different parameters for a new run
    occupancy_grid.setTo(cv::Scalar(255));
    delta_v_x = 0;
    position_y = 149;
    target = 202;
    target_vector = {};
    slow_x = 0;

    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = 0;
    acc_cmd.y = 0;
    pub_acc_cmd_.publish(acc_cmd);
    
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_ = {};
}
