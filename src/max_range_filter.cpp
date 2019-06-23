/*
 *	max_range_filter.cpp
 *
 *	filter of SQ-LiDAR max range
 *
 * 	author : Yoshitaka Nagai
 */

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLHeader.h>


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;


class RangeFilter
{
	public:
		RangeFilter();
		
		void max_range_filter(void);
		void pc_callback(const sensor_msgs::PointCloud2Ptr&);
		CloudIPtr pass_through(CloudIPtr, float);

	private:
		ros::Subscriber sub;
		ros::Publisher pub;
	
		sensor_msgs::PointCloud2 pub_pc;

		CloudIPtr input_pc_ {new CloudI};
		CloudIPtr filtered_pc_ {new CloudI};

		bool pc_callback_flag = false;

		float Hz = 100;
		double border_range;
		float max_intensity = 0.0;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "range_filter_UTM-30LX-FEW");

	RangeFilter range_filter;
	range_filter.max_range_filter();

	return 0;
}


RangeFilter::RangeFilter(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	nh.getParam("border_range", border_range);

	sub = n.subscribe("/lidar_pc", 10, &RangeFilter::pc_callback, this);
	pub = n.advertise<sensor_msgs::PointCloud2>("/range_filtered_pc", 10);
}


void RangeFilter::max_range_filter(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		if(pc_callback_flag){
			filtered_pc_ = pass_through(input_pc_, max_intensity);
			filtered_pc_->header.stamp = input_pc_->header.stamp;
			filtered_pc_->header.frame_id = filtered_pc_->header.frame_id;

			pcl::toROSMsg(*filtered_pc_, pub_pc);
			pub.publish(pub_pc);
			pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void RangeFilter::pc_callback(const sensor_msgs::PointCloud2Ptr &msg)
{
	pcl::fromROSMsg(*msg, *input_pc_);
	
	for(auto& pt : input_pc_->points){
		float range = sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
		if(range > (float)border_range){
			pt.intensity = -999.9;
		}
		if(pt.intensity > max_intensity){
			max_intensity = pt.intensity;
		}
	}

	pc_callback_flag = true;
}


CloudIPtr RangeFilter::pass_through(CloudIPtr pc_input_, float intensity_max)
{
	pcl::PassThrough<PointI> pass;
	CloudIPtr pc_filtered_ {new CloudI};
	pass.setInputCloud(pc_input_);
	pass.setFilterFieldName("intensity");
	pass.setFilterLimits(0, intensity_max);
	pass.filter(*pc_filtered_);

	return pc_filtered_;
}

