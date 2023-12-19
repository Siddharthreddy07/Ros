#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
const double pi = 3.14159265358979323846;
const float max = 30;
int a;
class nav{
  public:

double lat1;
double lon1;
double latitude;
double longitude;
double deltaLat;
double deltaLon;
double bearing;
double R = 6371;
double ang;
double angle_rad;
double angle_error;
double dist;


double angle(double lat1,double long1,double latitude, double longitude) {
  deltaLat= (latitude-lat1)*pi/180;
  deltaLon= (longitude-long1)*pi/180;
  lat1 = lat1*pi/180;
  long1 = long1*pi/180;
  latitude = latitude*pi/180;
  double y = sin(deltaLon) * cos(latitude);
  double x = cos(lat1) * sin(latitude) - sin(lat1) * cos(latitude) * cos(deltaLon);
  bearing = atan2(y,x);
  bearing = 360 -bearing*180/pi;
  return bearing;
}
double distance( double lat1,double long1,double latitude, double longitude) {
  deltaLat = (latitude-lat1)*pi/180;
  deltaLon = (longitude-long1)*pi/180;
  lat1 = lat1*pi/180;
  latitude = latitude*pi/180;
  double a = pow(sin(deltaLat / 2), 2) + pow(sin(deltaLon / 2), 2) * cos(lat1) * cos(latitude);
  double c = 2 * asin(sqrt(a));
  return R * c;
}
};
class imu{
private:
struct quaternion{
    double x,y,z,w;
};
public:
quaternion quat;
double yaw;

};
class lidar{
public:
double region[5]={};
};

nav coords;
imu euler;
lidar laser;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
 euler.quat.x = msg->orientation.x;
  euler.quat.y = msg->orientation.y;
  euler.quat.z = msg->orientation.z;
  euler.quat.w = msg->orientation.w;
  double t1 = 2 * (euler.quat.w * euler.quat.z + euler.quat.x * euler.quat.y);
  double t2 = 1 - 2 * (euler.quat.y * euler.quat.y + euler.quat.z * euler.quat.z);
  euler.yaw = std::atan2(t1, t2);
};

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  coords.lat1 = msg->latitude;
  coords.lon1 = msg->longitude;
}
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
laser.region[0] = std::min(*min_element(msg->ranges.begin(),msg->ranges.begin()+108),max);
laser.region[1] = std::min(*min_element(msg->ranges.begin()+108,msg->ranges.begin()+216),max);
laser.region[2]= std::min(*min_element(msg->ranges.begin()+216,msg->ranges.begin()+324),max);
laser.region[3]= std::min(*min_element(msg->ranges.begin()+324,msg->ranges.begin()+432),max);
laser.region[4] = std::min(*min_element(msg->ranges.begin()+432,msg->ranges.begin()+540),max);
}
geometry_msgs::Twist go_to_location()
{   geometry_msgs::Twist msg;
     coords.ang = coords.angle(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
    coords.angle_rad = coords.ang*pi/180;
    coords.dist = coords.distance(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
    coords.angle_error = (coords.angle_rad-euler.yaw);

   if(fabs(coords.dist>0.0005)){
   {
    if(coords.angle_error>pi)
        coords.angle_error = coords.angle_error- 2*pi;
    else if(coords.angle_error<-pi)
        coords.angle_error = 2*pi + coords.angle_error; 
   }
        if(fabs(coords.angle_error)>0.01){
        msg.angular.z = coords.angle_error;
        if(coords.angle_error<0.5){
          msg.linear.x=50*coords.dist;
          msg.angular.z = coords.angle_error;
        }
    }
      else{
        msg.angular.z= 0;        
        if(fabs(coords.dist)>0.0005)
            msg.linear.x = 50*coords.dist;
        else
            msg.linear.x = 0.0;
        }
        return msg;
    }
}
geometry_msgs::Twist turn_left()
{
  coords.dist = coords.distance(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
  if(fabs(coords.dist>0.0005)){
    geometry_msgs::Twist msg;
    msg.angular.z = 1;
    return msg;
  }
}

geometry_msgs::Twist wall_follower()
{coords.dist = coords.distance(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
  if(fabs(coords.dist>0.0005)){
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    return msg;
}
}
int d=3;
int take_action(){
if (laser.region[2] > d  && laser.region[1] > d && laser.region[3] > d)
{
  std::cout<<"no obstacles";
    a=0;
}

else if (laser.region[2] < d && laser.region[1] > d && laser.region[3] > d)
{
  std::cout<<"front";
    a=1;
}
else if (laser.region[2] > d && laser.region[1]  > d && laser.region[3] < d)
{
  std::cout<<"fright";
  a=0;
}
else if (laser.region[2] > d && laser.region[1]  < d && laser.region[3] > d)
{
  std::cout<<"fleft";
 a=2;
}
else if (laser.region[2] < d && laser.region[1]  > d && laser.region[3] < d) 
{
 std::cout<<"front and fright";
  a=1;
}
else if (laser.region[2] < d && laser.region[1]  < d && laser.region[3] > d)
{
  std::cout<<"front and fleft";
 a=1;
}
else if (laser.region[2] < d && laser.region[1]  < d && laser.region[3] < d)
{
  std::cout<<"front,fleft and fright";
a=1;
}
else if (laser.region[2] > d && laser.region[1]  < d && laser.region[3] < d)
{
std::cout<<"fleft and fright";
 a=0;
}
return a;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_trav");
  ros::NodeHandle n;
  ros::Subscriber trav_imu_sub = n.subscribe("/imu", 1, imu_callback);
  ros::Subscriber trav_gps_sub = n.subscribe("/gps/fix", 1, gps_callback);
  ros::Subscriber trav_Laser_sub = n.subscribe("/scan", 1, laser_callback);
  ros::Publisher trav_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist vel_msg;
  std::cout<<"input target GPS coordinates"<<std::endl;
  std::cin>>coords.latitude >>coords.longitude;
  double target_dist = coords.distance(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
  while (ros::ok()) {
    geometry_msgs::Twist vel_msg;
    coords.ang = coords.angle(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
   coords.angle_rad = coords.ang*pi/180;
    coords.dist = coords.distance(coords.lat1,coords.lon1,coords.latitude,coords.longitude);
   coords.angle_error = (coords.angle_rad-euler.yaw);
    take_action();
 
 switch(a){
    case 0:
    vel_msg=go_to_location();
    break;
    case 1:
    vel_msg=turn_left();
    break;
    case 2:
    vel_msg=wall_follower();
    break;
}
        trav_pub.publish(vel_msg);
        std::cout<<std::endl<<a<<" , "<<laser.region[3]<<" , "<<laser.region[2]<<" , "<<laser.region[1]<<std::endl;
        std::cout<< coords.angle_error<<"Target_Angle: " << coords.angle_rad-2*pi<< std::endl<<"Current_Angle: "<<euler.yaw <<std::endl<<"Distance_Left:"<<coords.dist<< std::endl<<"Current location: "<<coords.lat1<<","<<coords.lon1<<std::endl<<coords.angle_error<<std::endl;
        ros::spinOnce();
    }
  }





