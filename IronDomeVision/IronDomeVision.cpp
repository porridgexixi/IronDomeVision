// IronDomeVision.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <time.h>

using namespace std;
using namespace cv;

#define BALL_RADIUS 0.035
#define ABS_MAX 100000

void MapToRobotCoordinate(ballData& data);

int _tmain(int argc, _TCHAR* argv[])
{
	/************************************************************/
	// Initialization

	CKinectDepthReader kinect;
	HRESULT hr = kinect.InitializeDefaultSensor();
	if (FAILED(hr))
	{
		cerr << "failed to initialize kinect sensor" << endl;
		return -1;
	}

	Mat depthMat(kinect.m_height, kinect.m_width, CV_16UC1);
	CBallTracking ballTracking;

	//Communication
	zmq::context_t context(1);
	zmq::socket_t publisher(context, ZMQ_PUB);
	publisher.bind("tcp://*:4242");
	//publisher.connect("tcp://192.168.1.2:3883");

	/************************************************************/

	bool shutdown = false;
	double lastDepthTime = 0;
	while (!shutdown)
	{
		double depthTime;

		kinect.UpdateDepthMat(depthMat, depthTime);

		//std::cout << depthTime - lastDepthTime << std::endl;

		if (depthTime == lastDepthTime)
		{
			continue;
			//std::cout << "same frame" << std::endl;
		}
		else
		{
			lastDepthTime = depthTime;
		}

		ballTracking.UpdateResults(depthMat, depthTime, kinect.m_pCoordinateMapper, true);

		// Publish data (results stored in ballTracking.m_balls)
		std::vector<ballData> results = ballTracking.m_balls;
		//std::cout << "--";

		for (std::vector<ballData>::iterator it = results.begin(); it != results.end(); it++)
		{
			MapToRobotCoordinate(*it);
			if (abs(it->x) > ABS_MAX || abs(it->y) > ABS_MAX || abs(it->z) > ABS_MAX)
			{
				continue;
			}
			zmq::message_t message;
			std::string message_str;
			message_str = to_string(it->id) + " " +
				to_string(it->timestamp) + " " +
				to_string(it->x) + " " +
				to_string(it->y) + " " +
				to_string(it->z);
			message = zmq::message_t((void*)(&message_str), sizeof(message_str), NULL);
			zmq_send(publisher, message_str.c_str(), strlen(message_str.c_str()), 0);

			std::cout << message_str << std::endl;
		}

		//std::cout << depthTime << std::endl;


		if (cv::waitKey(1) == VK_ESCAPE)
		{
			shutdown = true;
		}
	}
	return 0;
}

void MapToRobotCoordinate(ballData& data)
{
	Mat T = (Mat_<double>(4, 4) <<
		-0.7675, -0.1089, 0.6633, 1.4638,
		0.6524, -0.1114, 0.7391, -1.3210,
		0.0271, 0.9741, 0.1440, 1.3860,
		0, 0, 0, 1.0000);

	Mat Pk(4, 1, CV_64FC1);
	Mat Pr(4, 1, CV_64FC1);

	double x_k = data.x;
	double y_k = data.y;
	double z_k = data.z;
	double dist = sqrt(x_k * x_k + y_k * y_k + z_k * z_k);
	x_k = x_k * (dist + BALL_RADIUS) / dist;
	y_k = y_k * (dist + BALL_RADIUS) / dist;
	z_k = z_k * (dist + BALL_RADIUS) / dist;
	Pk.at<double>(0, 0) = x_k;
	Pk.at<double>(0, 1) = y_k;
	Pk.at<double>(0, 2) = z_k;
	Pk.at<double>(0, 3) = 1;

	Pr = T * Pk;

	data.x = Pr.at<double>(0, 0);
	data.y = Pr.at<double>(0, 1);
	data.z = Pr.at<double>(0, 2);

	//std::cout << Pr << std::endl;
}