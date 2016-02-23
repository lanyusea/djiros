#ifndef DJI_SDK_READ_CAM_NODE_H
#define DJI_SDK_READ_CAM_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

#include <cstdio>
#include <functional>
#include <thread>
#include "djicam.h"

class DJISDKReadCamNode {
public:
	DJISDKReadCamNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private): 
		image_transport(nh), 
		camera_pub(image_transport.advertiseCamera("image_raw", 10)) {

		nh_private.param("output_image_color", output_image_color, false);
		nh_private.param("image_width", image_width, 1280);
		nh_private.param("image_height", image_height, 720);
		nh_private.param("output_image_width", output_image_width, 640);
		nh_private.param("output_image_height", output_image_height, 480);
	    tf_prefix = tf::getPrefixParam(nh_private);
		
		cam_info.header.frame_id = tf::resolve(tf_prefix, "camera");
		cam_info.height = output_image_height;
		cam_info.width = output_image_width;
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

		int ret = manifold_cam_init(GETBUFFER_MODE | TRANSFER_MODE);

		if (ret == -1)
		{
			printf("manifold init error \n");
			assert(false);
		}

		buffer = new unsigned char[image_width * image_height * 3];
		buffer2 = new unsigned char[image_width * image_height * 3];

		okay = true;
		image_thread = std::thread(std::bind(&DJISDKReadCamNode::feedImages, this));
	}

    void feedImages() {
		unsigned int nframe = 0;
		cv::Size image_raw_size(image_width, image_height);
		cv::Size image_output_size(output_image_width, output_image_height);
		cv::Mat image_raw(image_raw_size, CV_8UC3);
		cv::Mat image_output(image_output_size, output_image_color ? CV_8UC3 : CV_8U);

    	while (okay) {

    		int ret = manifold_cam_read(buffer, &nframe, 1);

			if (ret == -1) {
				okay = false;
				break;
			}

			if (output_image_color){
				NV12ToRGB(buffer, buffer2, image_width, image_height);
				memcpy(image_raw.data, buffer2, image_width * image_height * 3);
			} else {
				memcpy(image_raw.data, buffer, image_width * image_height);
			}
			cv::resize(image_raw, image_output, image_output_size);

			// Publish
    		ros::Time capture_time = ros::Time::now();

    		cv_bridge::CvImage cvi;
			cvi.header.stamp = capture_time;
			cvi.header.frame_id = cam_info.header.frame_id;
			if (output_image_color) {
				cvi.encoding = sensor_msgs::image_encodings::BGR8;
			} else {
				cvi.encoding = sensor_msgs::image_encodings::MONO8;
			}

			cvi.image = image_output;
			cam_info.header.seq = nframe;
			cam_info.header.stamp = capture_time;
			
			sensor_msgs::Image im;
            cvi.toImageMsg(im);
            camera_pub.publish(im, cam_info);

			ros::spinOnce();

			if (!manifold_cam_exit()) {
				okay = false;
				break;
			}

			usleep(1000);
    	}

		while (!manifold_cam_exit())
		{
			sleep(1);
		}
    }

	~DJISDKReadCamNode() {
		okay = false;
		delete [] buffer;
		delete [] buffer2;
	  	image_thread.join();
	}

private: 
	bool okay = false;
	bool output_image_color = false;
	int image_width, image_height;
	int output_image_width, output_image_height;
	unsigned char* buffer;
	unsigned char* buffer2;

	std::string tf_prefix;
	image_transport::ImageTransport image_transport;
	image_transport::CameraPublisher camera_pub;
	sensor_msgs::CameraInfo cam_info;
	std::thread image_thread;

	struct sRGB{
		int r;
		int g;
		int b;
	};

	sRGB YUVToRGB(int Y, int U, int V)
	{
		sRGB rgb;
		rgb.r = (int)(Y + 1.4075 * (V - 128));
		rgb.g = (int)(Y - 0.3455 * (U - 128) - 0.7169 * (V-128));
		rgb.b = (int)(Y + 1.779 * (U - 128));
		rgb.r = (rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
		rgb.g = (rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
		rgb.b = (rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
		return rgb;
	}

	unsigned char* NV12ToRGB(unsigned char* src, unsigned char* rgb, int width, int height) {
		int numOfPixel = width * height;
		int positionOfU = numOfPixel;
		int startY, step, startU, Y, U, V, index, nTmp;
		sRGB tmp;

		for (int i=0; i< height; i++) {
			startY = i * width;
			step = i / 2 * width;
			startU = positionOfU + step;
			for (int j = 0; j < width; j++){
				Y = startY + j;
				if(j % 2 == 0)
					nTmp = j;
				else
					nTmp = j - 1;
				U = startU + nTmp;
				V = U + 1;
				index = Y*3;
				tmp = YUVToRGB((int)src[Y], (int)src[U], (int)src[V]);
				rgb[index+0] = (char)tmp.b;
				rgb[index+1] = (char)tmp.g;
				rgb[index+2] = (char)tmp.r;
			}
		}
		return rgb;
	}
};

#endif

