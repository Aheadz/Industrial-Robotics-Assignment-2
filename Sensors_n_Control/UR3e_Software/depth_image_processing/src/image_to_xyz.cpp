#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>

// No need to use "using namespace std;"

typedef sensor_msgs::PointCloud2 SENS_POINTCLOUD_2;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PCL_XYZRGBA;
typedef geometry_msgs::PoseArray GEOM_POSE_ARRAY;
typedef geometry_msgs::Pose GEOM_POSE;

GEOM_POSE_ARRAY XYZ_array;

// Make the ROS publisher a local variable
const double MAX_DISTANCE = 0.32;
const double MIN_DISTANCE = 0.15;

void pointCloudCallback(const boost::shared_ptr<const SENS_POINTCLOUD_2> msg)
{
  PCL_XYZRGBA::Ptr xyz_rgba_cloud(new PCL_XYZRGBA);
  pcl::fromROSMsg(*msg, *xyz_rgba_cloud);

  XYZ_array.poses.clear();
  for (size_t i = 0; i < xyz_rgba_cloud->points.size(); i++)
  {
    const pcl::PointXYZRGBA& point = xyz_rgba_cloud->points[i];
    GEOM_POSE pose;

    if (point.z > MIN_DISTANCE && point.z < MAX_DISTANCE)
    {
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = point.z;
    }
    else
    {
      pose.position.x = 0;
      pose.position.y = 0;
      pose.position.z = 0;
    }

    XYZ_array.poses.push_back(pose);
  }

  static ros::Publisher pub = msg->header.stamp.toSec() == ros::Time::now().toSec() ? pub : ros::NodeHandle().advertise<GEOM_POSE_ARRAY>("camera/XYZ_location_of_pixels", 1);
  pub.publish(XYZ_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl_to_publish_xyz_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<SENS_POINTCLOUD_2>("camera/depth_registered/points", 1, pointCloudCallback);

  ros::spin();
}