#include <ros/ros.h>
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros_simple/whycon_rosConfig.h>

image_transport::Publisher imdebug;
ros::Publisher command_pub;
ros::Publisher pose_pub;

CCircleDetect *detector;
CTransformation *transf;

SSegment currentSegment;
SSegment lastSegment;
CRawImage *grayImage;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;

int circleDetections = 0;
int maxCircleDetections = 10;
float circleDiameter = 0.07;

//parameter reconfiguration
void reconfigureCallback(whycon_ros_simple::whycon_rosConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure request: %lf %lf %lf %lf %lf %lf", config.circleDiameter, config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
	circleDiameter = config.circleDiameter/100.0;
	detector->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
}

//image callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//resize image if needed 
	if (grayImage->bpp != msg->step/msg->width || grayImage->width != msg->width || grayImage->height != msg->height){
		delete grayImage;
		ROS_DEBUG("Readjusting grayImage format from %ix%i %ibpp, to %ix%i %ibpp.",grayImage->width,grayImage->height,grayImage->bpp,msg->width,msg->height,msg->step/msg->width);
		grayImage = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}

	memcpy(grayImage->data,(void*)&msg->data[0],msg->step*msg->height);

	lastSegment = currentSegment;
	currentSegment = detector->findSegment(grayImage,lastSegment);

	//if the circle is visible 
	if (currentSegment.valid)
	{
		STrackedObject o = transf->transform(currentSegment);
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = -o.y;
		pose.pose.position.y = -o.z;
		pose.pose.position.z = o.x;
		ROS_INFO("Circle detected at %.2f %.2f %.3f\n",-o.y,-o.z,o.x);
		pose_pub.publish(pose);	
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "whycon");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	grayImage = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	detector = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	//initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<whycon_ros_simple::whycon_rosConfig> server;
	dynamic_reconfigure::Server<whycon_ros_simple::whycon_rosConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);

	transf = new CTransformation(circleDiameter);
	image_transport::Subscriber subimGray = it.subscribe("/head_xtion/rgb/image_mono", 1, grayImageCallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/circlePosition", 1);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
}

