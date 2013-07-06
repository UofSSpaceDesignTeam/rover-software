/*
 * stream.cpp
 *
 *  Created on: 2013-05-22
 *      Author: curtis
 */
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


#include <cstdlib>
#include <iostream>
#include <vector>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::ModelCoefficients Plane;


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
using namespace std;



// PARAMETERS - TODO DON'T USE GLOBAL VARIABLES
int FRAMERATE = 30;
double DOWNSAMPLE_FRAMES = 0.02; // final resolution of frames



//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};



void downSample(PointCloud::ConstPtr input,
		PointCloud::Ptr output, float resolution)
{
	// Create the filtering object
	  pcl::VoxelGrid<PointT> sor;

	  sor.setInputCloud (input);
	  sor.setLeafSize (resolution, resolution, resolution);
	  sor.filter (*output);
}

// SEGMENTS IMAGE INTO PLANAR COMPONENTS

// UTILITY METHODS

// from the given input, removes all points more than 'threshold'
// meters above (in negative Y direction) the given planeW
void removePointsAbovePlane(Plane plane,
						    PointCloud::Ptr input,
						    PointCloud::Ptr output,
						    double threshold){

}

// true if plane1 is orthogonal to plane2 to within 'tolerance' radians
bool isOrthogonal(Plane plane1,
				  Plane plane2,
				  double tolerance){
	return true;
}

// true if the plane is parallel to the plane formed by the X and Z axes
// to within 'tolerance' radians
bool isGround(Plane plane,
				double tolerance){
	return true;
}

// return the given point from the point of view of an observer standing
// on the plane immediately below the camera (0,0,0)
PointT relativeToGround(PointT point, Plane plane){
	// find the shortest vector from the camera to the plane, then substract
	// it from the point. TADA
	return *(new PointT);
}

// extracts planes from the input point cloud
// points must be within 'precision' meters of a plane to be considered
// part of it; extraction stops when 'fraction' or less of the original points
// remain; the remaining points are returned in the 'remainder' pointcloud

// if ground isn't NULL, only keep planes that are orthogonal to within
// groundCheckPrecision radians of the ground. If plane is not orthogonal,
// put all its points into the remainder
void extractPlanes(PointCloud::Ptr input,
				     float precision,
				     float fraction,
				     int maxPlanes,
				     PointCloud::Ptr remainder,
				     vector<Plane> planes,
				     PointCloud::Ptr ground,
				     float groundCheckPrecision = 0)
{
 /*
  *  Plane coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (precision);
  */

}

void segment (PointCloud::ConstPtr input)
{
  PointCloud::Ptr cloud_temp (new PointCloud),
		  	  	  obstacles (new PointCloud),
		  	  	  cloud_remainder (new PointCloud);

  *cloud_temp = *input;

  // read the precision
  float precision = 0.03;
  float fraction = 0.1;
  double tolerance = 5 * 3.14159 / 180; // 5 degrees
  double threshold = 0.5;
  double obstacleResolution = 0.05;

  vector<Plane> planes;

  // blindly extract some planes, man
  extractPlanes(cloud_temp, precision, fraction,
		  	  	6, cloud_remainder, planes, NULL);


  // find the ground plane
  Plane ground;
  bool foundGround = false;
  for (int i = 0; i < planes.size() && !foundGround; i++){
	  if (isGround(planes[i], tolerance)){
		  ground = planes[i];
		  foundGround = true;
	  }
  }

  if (foundGround){

	  // filter points too far above ground
	  removePointsAbovePlane(ground, input, cloud_temp, threshold);

	  // extract all planes that are orthogonal or parallel to ground
	  planes.clear();
	  extractPlanes(cloud_temp, precision, fraction,
			  	  	4, cloud_remainder, planes, ground, tolerance);

	  // get the obstacles
	  downSample(cloud_remainder, obstacles, obstacleResolution);

	  for (PointCloud::iterator it = obstacles -> begin();
		   it != obstacles -> end();
		   it++){
		  PointT pt = *it;
		  PointT obstacle = relativeToGround(pt, ground);

		  // OUTPUT OBSTACLE TO JAVA
	  }
  }

  // OUTPUT PLANES TO JAVA
}




class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer(){
			counter = 1;
			firstFrame = true;
	}
    int counter;
    bool firstFrame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentModel;




	void cloud_cb_(const PointCloud::ConstPtr &cloud)
	{
		//cerr << "Got frame " << counter << endl;
		if (counter % FRAMERATE == 0 )
		{
			cerr << "It's a valid frame; processing " << endl;
			//Create the downsampled pointcloud for each frame we want to capture
			PointCloud::Ptr downsampled(
					new PointCloud);
			downSample(cloud, downsampled, DOWNSAMPLE_FRAMES);

			segment(downsampled);
		}
		counter ++;
	}



	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();


		boost::function<void(const PointCloud::ConstPtr&)> f =
				boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);

		interface->start();

		//Blocking call to keep going until string is read in
	    string s;
	    cin >> s;

		interface->stop();
	}
};

int main(int argc, char ** argv) {
	if (argc == 1)
	{}
	else if (argc == 5)
	{
		// read in arguments
		/*stringstream ss1(argv[1]);
		ss1 >> FRAMERATE;
		stringstream ss2(argv[2]);
		ss2 >> DOWNSAMPLE_FRAMES;

		stringstream ss3(argv[3]);
		ss3 >> DOWNSAMPLE_REG;

		stringstream ss4(argv[4]);
		ss4 >> MAX_CORRESPONDENCE_DISTANCE;*/

	} else {
		cerr << "Usage : stream <framerate> <downsample frames> <downsample reg> <max corr distance> "
				<< endl;
	}



	SimpleOpenNIViewer v;
	v.run();
	return 0;
}



