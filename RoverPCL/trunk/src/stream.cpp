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
using namespace std;


// PARAMETERS - TODO DON'T USE GLOBAL VARIABLES
int FRAMERATE = 1;
double DOWNSAMPLE_FRAMES = 0.02; // final resolution of frames
double DOWNSAMPLE_REG = 0.05; //resolution at which frames is compared
double MAX_CORRESPONDENCE_DISTANCE = 0.3;


int NUM_ITERATIONS = 40;
double TRANSFORMATION_EPSILON = 1e-6;


void downSample(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr output, float resolution)
{

	// Create the filtering object
	  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;

	  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2 ());
	  sensor_msgs::PointCloud2::Ptr cloud2(new sensor_msgs::PointCloud2 ());
	  pcl::toROSMsg(*input, *cloud);



	  sor.setInputCloud (cloud);
	  sor.setLeafSize (resolution, resolution, resolution);
	  sor.filter (*cloud2);
	  pcl::fromROSMsg(*cloud2, *output);
}


class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() :
			viewer("PCL OpenNI Viewer") {
			counter = 1;
	}
    int counter;


	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);

		downSample(cloud, downsampled, DOWNSAMPLE_FRAMES);
		if (counter % FRAMERATE == 0 ) {
			if (!viewer.wasStopped())
				viewer.showCloud(downsampled);
			//stringstream ss;
			//ss << counter / frameFrequency << ".pcd";
			//pcl::io::savePCDFile (ss.str (), *cloud, true);
		}
		counter ++;


	}

	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
				boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);

		interface->start();

		while (!viewer.wasStopped()) {
			boost::this_thread::sleep(boost::posix_time::seconds(1));
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



