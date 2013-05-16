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

class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() :
			viewer("PCL OpenNI Viewer") {
			counter = 1;
	}
    int counter;
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
		counter ++;

		if (counter % 10 == 0) {
			std::stringstream ss;
			ss << counter / 10 << ".pcd";
			pcl::io::savePCDFile (ss.str (), *cloud, true);
		}
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
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

int main() {
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}
