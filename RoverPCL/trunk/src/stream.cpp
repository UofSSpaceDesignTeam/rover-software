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


#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <iostream>
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
using namespace std;



// PARAMETERS - TODO DON'T USE GLOBAL VARIABLES
int FRAMERATE = 1;
double DOWNSAMPLE_FRAMES = 0.02; // final resolution of frames
double DOWNSAMPLE_REG = 0.05; //resolution at which frames is compared
double MAX_CORRESPONDENCE_DISTANCE = 0.3;


int NUM_ITERATIONS = 40;
double TRANSFORMATION_EPSILON = 1e-6;

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

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
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


double pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
		PointCloud::Ptr output)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);

  if (DOWNSAMPLE_REG < DOWNSAMPLE_FRAMES)
  {
	  downSample(cloud_src, src, DOWNSAMPLE_REG);
	  downSample(cloud_tgt, tgt, DOWNSAMPLE_REG);
  } else {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (TRANSFORMATION_EPSILON);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (MAX_CORRESPONDENCE_DISTANCE);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (NUM_ITERATIONS);
  //for (int i = 0; i < 20; ++i)
  //{
    //PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);

    double fitness = reg.getFitnessScore(0.1);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    //if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      //reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    //prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  //}

	//
  // Get the transformation from target to source
  //targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_src, *output, Ti);
  return fitness;
 }



class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() :
			viewer("PCL OpenNI Viewer") {
			counter = 1;
			firstFrame = true;
	}
    int counter;
    bool firstFrame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentModel;




	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		cout << "Got frame " << counter << endl;
		if (counter % FRAMERATE == 0 )
		{
			cout << "It's a valid frame; processing " << endl;
			//Create the downsampled pointcloud for each frame we want to capture
			pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
			downSample(cloud, downsampled, DOWNSAMPLE_FRAMES);

			if (firstFrame == true){
				firstFrame = false;
				currentModel = downsampled;
			} else {
				// merge the frame into the current model
				double fitness = pairAlign(downsampled, currentModel, downsampled);



				//downsampled contains the aligned frame

				//Merge downsampled with currentModel
				//pcl::PointCloud<pcl::PointXYZ>::Ptr newModel(new pcl::PointCloud<pcl::PointXYZ>);

				*currentModel += *downsampled;

				//Downsample the new model
				downSample(currentModel, currentModel, DOWNSAMPLE_FRAMES);

				cout << "Merged with fitness : " << fitness << endl;
				cout << currentModel -> width * currentModel -> height << endl;

				if (!viewer.wasStopped())
					viewer.showCloud(currentModel);
				cout << "done processing " << endl;

			}


		}
		counter ++;
	}



	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();


		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
				boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);

		//interface->start();

		bool started = false;

		while (!viewer.wasStopped()) {
			char c;
			cin >> c;
			if (started) interface -> stop();
			else interface -> start();
			started = !started;
			cout << " Started: " << started << endl;
			//boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		interface->stop();
	}

	pcl::visualization::CloudViewer viewer;
};

int main(int argc, char ** argv) {
	if (argc == 1)
	{}
	else if (argc == 5)
	{
		// read in arguments
		stringstream ss1(argv[1]);
		ss1 >> FRAMERATE;
		stringstream ss2(argv[2]);
		ss2 >> DOWNSAMPLE_FRAMES;

		stringstream ss3(argv[3]);
		ss3 >> DOWNSAMPLE_REG;

		stringstream ss4(argv[4]);
		ss4 >> MAX_CORRESPONDENCE_DISTANCE;

	} else {
		cout << "Usage : stream <framerate> <downsample frames> <downsample reg> <max corr distance> "
				<< endl;
	}



	SimpleOpenNIViewer v;
	v.run();
	return 0;
}



