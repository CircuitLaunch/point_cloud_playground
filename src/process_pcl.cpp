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
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;


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

        // Perform voxel grid downsampling filtering
        pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud_msg);
        sor.setLeafSize (0.01, 0.01, 0.01);
        sor.filter (* cloudVoxel);

        //FILTER ALONG CAM Z AXIS
		pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		// define a PassThrough
		pcl::PassThrough<pcl::PCLPointCloud2> pass;
		// set input to cloudVoxel
		pass.setInputCloud(cloud_msg);
		// filter along z-axis
		pass.setFilterFieldName("z");
		// set z-limits
		pass.setFilterLimits(0.2, 1.0);
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
        extract.setNegative (true);
        extract.filter (*cloud);

        //this->publisher.publish (*cloud);

        //CLOUD CLUSTERING SECTION
        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (0.005); // 2cm
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (2500);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster_indices);

        // declare the output variable instances
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 outputPCL;

        // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {

            // create a new clusterData message object
            //obj_recognition::ClusterData clusterData;

            // create a pcl object to hold the extracted cluster
            pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr (cluster);

            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
            clusterPtr->points.push_back(cloud->points[*pit]);

                }


            // log the position of the cluster
            //clusterData.position[0] = (*cloudPtr).data[0];
            //clusterData.position[1] = (*cloudPtr).points.back().y;
            //clusterData.position[2] = (*cloudPtr).points.back().z;
            //std::string info_string = string(cloudPtr->points.back().x);
            //printf(clusterData.position[0]);

            // convert to pcl::PCLPointCloud2
            pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

            // Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);
            output.header.frame_id = "depth_camera_link" ;
            
            this->publisher.publish (output);
            //std::cerr << "Publishing cluster: " << *it << " ";
            ros::Duration(0.1).sleep(); // sleep for half a second

            // add the cluster to the array message
            //clusterData.cluster = output;
            //CloudClusters.clusters.push_back(output);

        }

  
        // //CYLINDER SEGMENTATION SECTION////

        // //pcl::ExtractIndices<PointT> extract;
        // pcl::ExtractIndices<pcl::Normal> extract_normals;
        // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        // pcl::NormalEstimation<PointT, pcl::Normal> ne;

        // // Estimate point normals
        // ne.setSearchMethod (tree);
        // ne.setInputCloud (cloud);
        // ne.setKSearch (50);
        // ne.compute (*cloud_normals);

        // pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg1; 
        // // Create the segmentation object for cylinder segmentation and set all the parameters
        // seg1.setOptimizeCoefficients (true);
        // seg1.setModelType (pcl::SACMODEL_CYLINDER);
        // seg1.setMethodType (pcl::SAC_RANSAC);
        // seg1.setNormalDistanceWeight (0.1);
        // seg1.setMaxIterations (10000);
        // seg1.setDistanceThreshold (0.05);
        // seg1.setRadiusLimits (0.0, 0.10);
        // seg1.setInputCloud (cloud);
        // seg1.setInputNormals (cloud_normals);

        // // Obtain the cylinder inliers and coefficients
        // pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        // pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
        // seg1.segment (*inliers_cylinder, *coefficients_cylinder);

        // if (inliers->indices.size () == 0)
        // {
        //     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        // }

        // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        // extract.setInputCloud (cloud);
        // extract.setIndices (inliers_cylinder);
        // extract.setNegative (false);
        // extract.filter (*cloud);

        // this->publisher.publish (*cloud);

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