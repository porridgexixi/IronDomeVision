// IronDomeVision.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

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
	publisher.bind("tcp://*:5563");

	/************************************************************/

	bool shutdown = false;
	while (!shutdown)
	{
		double depthTime;

		kinect.UpdateDepthMat(depthMat, depthTime);

		ballTracking.UpdateResults(depthMat, depthTime, kinect.m_pCoordinateMapper, true);
		
		// Publish data (results stored in ballTracking.m_balls)
		std::vector<ballData> results = ballTracking.m_balls;
		for (std::vector<ballData>::iterator it = results.begin(); it != results.end(); it++)
		{
			zmq::message_t message;
			std::string message_str;
			message_str =	to_string(it->id) + " " +
							to_string(it->timestamp) + " " +
							to_string(it->x) + " " +
							to_string(it->y) + " " +
							to_string(it->z);
			message = zmq::message_t((void*)(&message_str), sizeof(message_str), NULL);
			publisher.send(message);					   
		}
		

		if (cv::waitKey(1) == VK_ESCAPE)
		{
			shutdown = true;
		}
	}
	return 0;
}

