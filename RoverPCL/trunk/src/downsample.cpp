#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


void downSample(sensor_msgs::PointCloud2::Ptr input, sensor_msgs::PointCloud2::Ptr output, float resolution)
{
	// Create the filtering object
	  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	  sor.setInputCloud (input);
	  sor.setLeafSize (resolution, resolution, resolution);
	  sor.filter (*output);
}


int main (int argc, char** argv)
{
	if (argc != 4) {
		std::cerr << "Usage: downsample <resolution in m> <original pcd> <destination pcd>";
	}

  // read in the resolution
  float resolutionCm;
  sscanf(argv[1], "%f", &resolutionCm);

  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ());
  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[2], *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  downSample(cloud, cloud_filtered, resolutionCm);


  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write (argv[3], *cloud_filtered,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
