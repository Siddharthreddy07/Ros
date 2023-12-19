#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

double roll, pitch, yaw;

class Navigation
{
public:
    float lat1;
    float lon1;
    float bright;
    float latitude;
    float longitude;
    float current_target;
    float num_of_samples;
    float distance;

    geometry_msgs::Twist cmd_vel;
    double region[5] = {};
};

class Lidar
{
public:
    double region[5] = {};
};

Navigation coords;
Lidar laser;

class AutoTrav
{
public:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher cmd_vel_pub;

    AutoTrav()
    {
        gps_sub = nh.subscribe("/gps/fix", 100, &AutoTrav::gpsCallback, this);
        imu_sub = nh.subscribe("/imu", 100, &AutoTrav::imuCallback, this);
        lidar_sub = nh.subscribe("/converted_laserscan", 100, &AutoTrav::laserCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        tf::Quaternion orientation_q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf::Matrix3x3(orientation_q).getRPY(roll, pitch, yaw);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        coords.lat1 = msg->latitude;
        coords.lon1 = msg->longitude;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {   
        laser.region[0] = std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 72), 10.0f);
        laser.region[1] = std::min(*std::min_element(msg->ranges.begin() + 73, msg->ranges.begin() + 144), 10.0f);
        laser.region[2] = std::min(*std::min_element(msg->ranges.begin() + 145, msg->ranges.begin() + 216), 10.0f);
        laser.region[3] = std::min(*std::min_element(msg->ranges.begin() + 217, msg->ranges.begin() + 288), 10.0f);
        laser.region[4] = std::min(*std::min_element(msg->ranges.begin() + 289, msg->ranges.begin() + 362), 10.0f);
    }

    float distance(float latitude, float longitude)
    {
        int R = 6371e3;
        float phi1 = coords.lat1 * M_PI / 180;
        float phi2 = latitude * M_PI / 180;
        float deltaLat = (latitude - coords.lat1) * M_PI / 180;
        float deltaLon = (longitude - coords.lon1) * M_PI / 180;
        float a = std::sin(deltaLat / 2) * std::sin(deltaLat / 2) + std::cos(phi1) * std::cos(phi2) * std::sin(deltaLon / 2) * std::sin(deltaLon / 2);
        float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        float distance1 = R * c;
        return distance1;
    }

    float angle(float latitude, float longitude, float currentlatitude, float currentlongitude)
    {
        int R = 6371e3;
        float phi1 = (currentlatitude) * M_PI / 180;
        float phi2 = (latitude) * M_PI / 180;
        float deltaLat = (latitude - currentlatitude) * M_PI / 180;
        float deltaLon = (longitude - currentlongitude) * M_PI / 180;
        float y = std::sin(deltaLon) * std::cos(phi2);
        float x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(deltaLon);
        float theta = M_PI_2 - std::atan2(y, x);
        float target1 = std::fmod((theta * 180 / M_PI + 360), 360);
        return target1;
    }

    void straight()
    {
        if (coords.distance > 0.5)
        {
            coords.cmd_vel.linear.x = 1;
            coords.cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(coords.cmd_vel);
            ROS_INFO("going straight");
        }
        else
        {
            coords.cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(coords.cmd_vel);
            ROS_INFO("REACHED");
        }
    }

    void turn()
    {
        if (std::abs(coords.current_target - yaw) > 0.07)
        {
            if (coords.current_target - yaw > 0)
            {
                coords.cmd_vel.angular.z = 0.7 * std::abs(coords.current_target - yaw);
                coords.cmd_vel.linear.x = 0.0;
                ROS_INFO("turning left");

                cmd_vel_pub.publish(coords.cmd_vel);
            }
            else
            {
                coords.cmd_vel.angular.z = -0.7 * std::abs(coords.current_target - yaw);
                coords.cmd_vel.linear.x = 0.0;
                ROS_INFO("turning right");
                cmd_vel_pub.publish(coords.cmd_vel);
            }
        }
        else
        {
            coords.cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(coords.cmd_vel);
            //ROS_INFO("GO STRAIGHT");
            straight();
        }
    }

    void handleCases()
    {
        float X = 2.0;
        if (laser.region[3] < (X + 0.5) && laser.region[2] > X && laser.region[1] > X)
        {
            ROS_INFO(" CASE 1 - Front left");
            coords.cmd_vel.linear.x = 0.5;
            coords.cmd_vel.angular.z = -3.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] > X && laser.region[2] < (X + 0.5) && laser.region[1] > X)
        {
            ROS_INFO(" CASE 2 - Front");
            coords.cmd_vel.linear.x = 1.0;
            coords.cmd_vel.angular.z = -2.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] > X && laser.region[2] > X && laser.region[1] < (X + 0.5))
        {
            ROS_INFO(" CASE 3 - Front Right");
            coords.cmd_vel.linear.x = 0.5;
            coords.cmd_vel.angular.z = 2.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] > X && laser.region[2] < X && laser.region[1] < X)
        {
            ROS_INFO(" CASE 4 Front and Front Right");
            coords.cmd_vel.linear.x = 0.0;
            coords.cmd_vel.angular.z = 1.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] < X && laser.region[2] > X && laser.region[1] < X)
        {
            ROS_INFO(" CASE 5 Front left and Front right");
            coords.cmd_vel.linear.x = 1.0;
            coords.cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] < X && laser.region[2] < X && laser.region[1] > X)
        {
            ROS_INFO(" CASE 6 Front and Front Left");
            coords.cmd_vel.linear.x = 0.0;
            coords.cmd_vel.angular.z = -1.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else if (laser.region[3] < X && laser.region[2] < X && laser.region[1] < X)
        {
            ROS_INFO(" CASE 7 Front and Front Left and Front Right");
            coords.cmd_vel.linear.x = 0;
            coords.cmd_vel.angular.z = -1.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
        else
        {
            ROS_INFO("else case");
            coords.cmd_vel.angular.z = 0;
            coords.cmd_vel.linear.x = 1.0;
            cmd_vel_pub.publish(coords.cmd_vel);
        }
    }

    void run()
    {
        ros::Rate rate(10.0);

        ROS_INFO("Enter Latitude: ");
        std::cin >> coords.latitude;
        ROS_INFO("Enter Longitude: ");
        std::cin >> coords.longitude;

        while (ros::ok())
        {
            geometry_msgs::Twist cmd_vel;

            coords.current_target = angle(coords.latitude, coords.longitude, coords.lat1, coords.lon1) * M_PI / 180;
            coords.distance = distance(coords.latitude, coords.longitude);
            
            if (coords.current_target >= M_PI)
            {
                coords.current_target -= 2 * M_PI;
            }
            ROS_INFO("distance=%f", coords.distance);
            ROS_INFO("yaw=%f", yaw);
            ROS_INFO("target angle =%f", coords.current_target);

            if (coords.distance > 1)
            {
                float X = 2.0;
                if (laser.region[2] < X || laser.region[1] < X || laser.region[3] < X)
                {
                    handleCases();
                }
                else if ((laser.region[2] > X || laser.region[1] > X || laser.region[3] > X || laser.region[4] > X) && laser.region[0] < X)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 2.0;
                    
                }
                else if ((laser.region[2] > X || laser.region[1] > X || laser.region[3] > X || laser.region[0] > X) && laser.region[4] < X)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = -2.0;
                    
                }
                else
               {
                turn();
               }
            }
            else
            {
                cmd_vel.angular.z = 0;
                cmd_vel_pub.publish(cmd_vel);
                ROS_INFO("REACHED");
                straight();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_trav");
    AutoTrav auto_trav;
    auto_trav.run();

    return 0;
}
