#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;

typedef sensor_msgs::PointCloud2 SENS_POINTCLOUD_2;
typedef pcl::PCLPointCloud2 PCL_POINTCLOUD_2;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PCL_XYZRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCL_XYZRGB;
typedef sensor_msgs::Image SENS_IMAGE;

ros::Publisher pub;

vector<int> current_image(640 * 480 * 3, 0);
SENS_IMAGE current_image_1;

const double MAX_DISTANCE = 0.32;
const double MIN_DISTANCE = 0.15;

void PointCloudXYZRGBAtoXYZRGB(PCL_XYZRGBA &in, PCL_XYZRGB &out)
{
  out.width = in.width;
  out.height = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size(); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }


  for (size_t i = 0; i < out.points.size(); i++)
  {
    if (out.points[i].z > MAX_DISTANCE || out.points[i].z < MIN_DISTANCE || isnan(out.points[i].z))
    {
      int j = i*3;
      current_image_1.data.at(j) = 0;
      current_image_1.data.at(j+1) = 0;
      current_image_1.data.at(j+2) = 0;
    }
  }

  pub.publish(current_image_1);
}

void pointCloudCallback(const boost::shared_ptr<const SENS_POINTCLOUD_2> msg)
{
  // converting from pcl pointcloud2 to sensor_msgs pointcloud2
  PCL_POINTCLOUD_2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);

  // converting from sensor_msgs pointcloud2 to plc xyz_rgba
  PCL_XYZRGBA xyz_rgba_cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, xyz_rgba_cloud);

  // converting from sensor_msgs pointcloud2 to plc xyz_rgb
  PCL_XYZRGB XYZ_RGB_cloud;
  PointCloudXYZRGBAtoXYZRGB(xyz_rgba_cloud, XYZ_RGB_cloud);
  // pub.publish(XYZ_RGB_cloud);
}

void imageCallBack(const SENS_IMAGE msg)
{
  current_image_1=msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgb_image_within_range_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_1 = nh.subscribe<SENS_POINTCLOUD_2>("camera/depth_registered/points", 1, pointCloudCallback);
  ros::Subscriber sub_2 = nh.subscribe<SENS_IMAGE>("camera/color/image_rect_color", 1, imageCallBack);

  pub = nh.advertise<SENS_IMAGE>("camera/image_within_min_and_max_range", 1);

  ros::spin();
}