#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <iostream>


class lasersan_pc2_transformer
{
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  
public:
  lasersan_pc2_transformer()
  {}
  
  ~lasersan_pc2_transformer()
  {}
  
  void init()
  {
    sub_ = n_.subscribe("/spencer/sensors/laser_front/echo0", 1000, &lasersan_pc2_transformer::callback, this);
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/obstacles", 1000);
  }
  
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    if(!listener_.waitForTransform(
        msg->header.frame_id,
        "/odom",
        msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
        ros::Duration(2.0)))
    {
     std::cout<<"Error!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
     return;
    }
    
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("/odom",*msg,
	    cloud,listener_);

    ros::Time now = ros::Time::now();
    std::cout<<now.sec<<":"<<now.nsec<<", OK!!!!!!!!!!!!!!!!!"<<std::endl;
    
    pub_.publish(cloud);
  }
  
};



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lasersan2pc2");
  lasersan_pc2_transformer trans;
  trans.init();
  
  ros::spin();
  return 0;
}
