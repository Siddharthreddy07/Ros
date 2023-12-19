#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
const double pi = 3.14159265358979323846;
double yaw = 0.0;
double lat1,long1;
double angle(double lat1,double long1,double latitude, double longitude) {
  double deltaLat= (latitude-lat1)*pi/180;
  double deltaLon= (longitude-long1)*pi/180;
  lat1 = lat1*pi/180;
  long1 = long1*pi/180;
  latitude = latitude*pi/180;
  double y = sin(deltaLon) * cos(latitude);
  double x = cos(lat1) * sin(latitude) - sin(lat1) * cos(latitude) * cos(deltaLon);
  double bearing = atan2(y,x);
  bearing = 360 - bearing*180/pi;
  return bearing;
}

double distance( double lat1,double long1,double latitude, double longitude) {
  const double R = 6371; 
  double deltaLat= (latitude-lat1)*pi/180;
  double deltaLon= (longitude-long1)*pi/180;
  lat1 = lat1*pi/180;
  latitude = latitude*pi/180;
  double a = pow(sin(deltaLat / 2), 2) + pow(sin(deltaLon / 2), 2) * cos(lat1) * cos(latitude);
  double c = 2 * asin(sqrt(a));
  return R * c;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double w = msg->orientation.w;
  double t1 = +2.0 * (w * z + x * y);
  double t2 = +1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(t1, t2);
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  lat1 = msg->latitude;
  long1 = msg->longitude;
}
int main(int argc, char** argv) {
  double latitude,longitude;
  ros::init(argc, argv, "auto_trav");
  ros::NodeHandle n;
  ros::Subscriber trav_imu_sub = n.subscribe("/imu", 1, imu_callback);
  ros::Subscriber trav_gps_sub = n.subscribe("/gps/fix", 1, gps_callback);
  ros::Publisher trav_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist vel_msg;
  std::cout<<"input target GPS coordinates"<<std::endl;
  std::cin>>latitude >>longitude;
  double target_dist = distance(lat1,long1,latitude,longitude);
  while (ros::ok()) {
    geometry_msgs::Twist vel_msg;
    double ang = angle(lat1,long1,latitude,longitude);
    double angle_rad = ang*pi/180;
    double dist = distance(lat1,long1,latitude,longitude);
    double angle_error = (angle_rad-yaw);
    if(fabs(dist>0.0005)){
    if(angle_error>pi)
        angle_error = angle_error- 2*pi;
    else if(angle_error<-pi)
        angle_error = 2*pi + angle_error; 
    if(fabs(angle_error)>0.01){
        vel_msg.angular.z = angle_error;
        if(angle_error<0.5){
          vel_msg.linear.x=50*dist;
          vel_msg.angular.z = angle_error;
        }
    }
      else{
        vel_msg.angular.z= 0;        
        if(fabs(dist)>0.002)
            vel_msg.linear.x = 50*dist;
        else
            vel_msg.linear.x = 0.0;
        }
    }
    else{
    std::cout<<"Target reached"<<std::endl;
    }
        trav_pub.publish(vel_msg);
        std::cout<< "Target_Angle: " << angle_rad-2*pi<< std::endl<<"Current_Angle: "<< yaw << std::endl<<std::endl<<"Distance_Left:"<<dist<< std::endl<<"Current location: "<< lat1<<","<<long1<<std::endl;
        ros::spinOnce();
    }
  }
  
  
