#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>



#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class SubscribeProcessPublish
{
public:
    SubscribeProcessPublish()
    {
        // Assign subscriber
        this->subscriber = this->nh.subscribe("/k4a/points2", 5, &SubscribeProcessPublish::processLidarMeasurement, this);
        
	// Assign publisher
        this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("output", 1);
    }
    
	void 
        processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg)
    {
        std::cout << "Received lidar measurement with seq ID " << cloud_msg->header.seq << std::endl;

		// // define a new container for the data
		// pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
		// // define a voxelgrid
		// pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
		// // set input to cloud
		// voxelGrid.setInputCloud(cloud_msg);
		// // set the leaf size (x, y, z)
		// voxelGrid.setLeafSize(0.01, 0.01, 0.01);
		// // apply the filter to dereferenced cloudVoxel
		// voxelGrid.filter(*cloudVoxel);
		// //this->publisher.publish (*cloudVoxel);

        //FILTER ALONG CAM Z AXIS
		pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		// define a PassThrough
		pcl::PassThrough<pcl::PCLPointCloud2> pass;
		// set input to cloudVoxel
		pass.setInputCloud(cloud_msg);
		// filter along z-axis
		pass.setFilterFieldName("z");
		// set z-limits
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*floorRemoved);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromPCLPointCloud2(*floorRemoved, *cloud);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);


        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }

        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " " 
                                            << coefficients->values[3] << std::endl;




        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        //extract.setInputCloud (xyzCloudPtrFiltered);
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud);

        this->publisher.publish (*cloud);
        // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        // for (std::size_t i = 0; i < inliers->indices.size (); ++i)
        // for (const auto& idx: inliers->indices)
        //     std::cerr << idx << "    " << cloud->points[idx].x << " "
        //                             << cloud->points[idx].y << " "
        //                             << cloud->points[idx].z << std::endl;


		// // cascade the floor removal filter and define a container for floorRemoved	
		// pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		

				
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
};

int main(int argc, char** argv)
{
    // initialise the node
    ros::init(argc, argv, "process_lidar");

    std::cout << "Process_lidar node initialised" << std::endl;
    
	// create instance of PublishSubscribe
	SubscribeProcessPublish process;

    // handle ROS communication events
    ros::spin();

    return 0;
}