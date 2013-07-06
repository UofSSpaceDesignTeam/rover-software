#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


using namespace std;

// SEGMENTS IMAGE INTO PLANAR COMPONENTS
int
main (int argc, char** argv)
{
  if (argc != 4){
	  cout << "Usage: extract_indices <file name> <precision in m> <minimum planar fraction of image>" << endl;
	  return 1;
  }
  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (argv[1], *cloud_blob);

  // read the precision
  float precision = 0.01;
  std::stringstream ss(argv[2]);
  ss >> precision;
  float fraction = 0.3;
  std::stringstream ss2(argv[3]);
  ss2 >> fraction;

  cout << "Running with precision : " << precision << " min planar fraction " << fraction << endl;


  pcl::fromROSMsg (*cloud_blob, *cloud_filtered);

  pcl::PCDWriter writer;


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (precision);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While the appropriate fraction of the original cloud is still there
  while (cloud_filtered->points.size () > fraction * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl;

    stringstream ss;
    ss << "output" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // print the model coefficients
    cerr << "Plane coefficients : ";
    vector<float> coefs = coefficients -> values;
    for (int i = 0 ; i < coefs.size(); i++){
    	cerr << coefs[i] << " ";
    }
    cerr << endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  // print the non-planar part as the last cloud
  if (cloud_filtered -> points.size() > 0){
	  std::stringstream ss3;
	   ss << "output" << i << ".pcd";
	   writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false);
  }

  return (0);
}
