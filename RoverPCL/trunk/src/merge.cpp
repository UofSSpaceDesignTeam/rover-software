#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/*
 * merge.cpp
 *
 *  Created on: 2013-05-21
 *      Author: curtis
 */


int main (int argc, char **argv)
{
  if (argc < 4) std::cout << "usage: merge <dest> <source> [*]";


  PointCloud::Ptr mergedCloud(new PointCloud), temp(new PointCloud);
  // put the first source in the cloud
  pcl::io::loadPCDFile(argv[2], *mergedCloud);

  // merge the rest in
  for (int i = 3; i < argc; i++)
  {
      pcl::io::loadPCDFile (argv[i], *temp);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*temp, *temp, indices);

      *mergedCloud += *temp;
  }

 //save aligned pair, transformed into the first cloud's frame
 pcl::io::savePCDFile (argv[1], *mergedCloud, true);
}

