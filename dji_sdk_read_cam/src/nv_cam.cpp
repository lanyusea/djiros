#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>
#include <unistd.h>

#include "cv.h"
#include "highgui.h"
#include "djicam.h"

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE  IMAGE_W * IMAGE_H * 3
#define RESIZE_W 640
#define RESIZE_H 480

unsigned char buffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;

#define MODE (GETBUFFER_MODE | TRANSFER_MODE)

struct sRGB{
	int r;
	int g;
	int b;
};

sRGB yuvTorgb(int Y, int U, int V)
{
	sRGB rgb;
	rgb.r = (int)(Y + 1.4075 * (V-128));
	rgb.g = (int)(Y - 0.3455 * (U-128) - 0.7169*(V-128));
	rgb.b = (int)(Y + 1.779 * (U-128));
	rgb.r = (rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
	rgb.g = (rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
	rgb.b = (rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
	return rgb;
}

unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height) {
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int startY,step,startU,Y,U,V,index,nTmp;
	sRGB tmp;

	for(int i=0; i<height; i++){
		startY = i*width;
		step = i/2*width;
		startU = positionOfU + step;
		for(int j = 0; j < width; j++){
			Y = startY + j;
			if(j%2 == 0)
				nTmp = j;
			else
				nTmp = j - 1;
			U = startU + nTmp;
			V = U + 1;
			index = Y*3;
			tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
			rgb[index+0] = (char)tmp.b;
			rgb[index+1] = (char)tmp.g;
			rgb[index+2] = (char)tmp.r;
		}
	}
	return rgb;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"dji_image_raw");
	int ret,nKey;
	int nState = 1;
	int nCount = 1;

	int gray_or_rgb = 0;

	IplImage *pRawImg;
	IplImage *pImg;
	unsigned char *pData;

	ros::NodeHandle nh_private("~");
	nh_private.param("gray_or_rgb", gray_or_rgb, 0);

	printf("%d\n",gray_or_rgb);
	if (gray_or_rgb) {
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 3);
		pImg = cvCreateImage(cvSize(RESIZE_W, RESIZE_H), IPL_DEPTH_8U, 3);
		pData  = new unsigned char[IMAGE_W * IMAGE_H * 3];
	} else {
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 1);
		pImg = cvCreateImage(cvSize(RESIZE_W, RESIZE_H), IPL_DEPTH_8U, 1);
	}

	ros::NodeHandle node;
	image_transport::ImageTransport transport(node);
	image_transport::Publisher image_pub = transport.advertise("image_raw", 1);
	ros::Publisher caminfo_pub = node.advertise<sensor_msgs::CameraInfo>("camera_info",1);

	ros::Time time = ros::Time::now();

	cv_bridge::CvImage cvi;

	sensor_msgs::Image im;
	sensor_msgs::CameraInfo cam_info;
	cam_info.header.frame_id = "/camera";
	cam_info.height = IMAGE_H/2;
	cam_info.width = IMAGE_W/2;
	cam_info.distortion_model = "";
	cam_info.D.push_back(-0.1297646493949856);
	cam_info.D.push_back(0.0946885697670611);
	cam_info.D.push_back(-0.0002935002712265514);
	cam_info.D.push_back(-0.00022663675362156343);
	cam_info.D.push_back(0.0);
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;
	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

	ret = manifold_cam_init(MODE);
	if(ret == -1)
	{
		printf("manifold init error \n");
		return -1;
	}

	while(1)
	{
		ret = manifold_cam_read(buffer, &nframe, 1);

		if(ret == -1)
			break;
		if(gray_or_rgb){
			NV12ToRGB(buffer, pData, IMAGE_W, IMAGE_H);
			memcpy(pRawImg->imageData, pData, FRAME_SIZE);
		}else{
			memcpy(pRawImg->imageData, buffer, FRAME_SIZE / 3);
		}
		cvResize(pRawImg, pImg, CV_INTER_LINEAR);

		time = ros::Time::now();
		cvi.header.stamp = time;
		cvi.header.frame_id = "image";
		if(gray_or_rgb){
			cvi.encoding = "bgr8";
		}else{
			cvi.encoding = "mono8";
		}
		cvi.image = cv::cvarrToMat(pImg);
		cvi.toImageMsg(im);
		cam_info.header.seq = nCount;
		cam_info.header.stamp = time;
		caminfo_pub.publish(cam_info);
		image_pub.publish(im);

		ros::spinOnce();
		nCount++;

		if (!manifold_cam_exit()) {
			break;
		}

		usleep(1000);
	}

	while(!manifold_cam_exit())
	{
		sleep(1);
	}

	sleep(1);
	return 0;
}
