#pragma once
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>



using namespace cv;

typedef struct
{
	int id;
	double timestamp;
	double x;
	double y;
	double z;
}ballData;

class CBallTracking
{
public:
	CBallTracking();
	~CBallTracking();

	void SetSimpleBlobDetectorParams(SimpleBlobDetector::Params a_params);

	void UpdateResults(Mat a_depthMat, double a_depthTime, ICoordinateMapper* a_mapper, bool a_drawResults = true);

	std::vector<ballData> m_balls;
	std::vector<ballData> m_balls_transform;

private:
	std::vector<ballData> m_bufferBalls; // remember balls appeared within specific time intervals
	int m_id;
	double m_lastDetectTime;
	int m_lastBallsCount;

	SimpleBlobDetector::Params m_params;

	SimpleBlobDetector m_blobDetector;

	std::vector<KeyPoint> m_keyPoints;

	std::vector<CameraSpacePoint> m_cameraPoints;

	void DrawResults(Mat a_resultImg);
	int GetCameraSpaceCoordinate(Mat a_depthMap, ICoordinateMapper* a_mapper, CameraSpacePoint* a_cameraPoints);
	void GetFinalResults(double a_depthTime);
	void UpdateBufferBalls(double a_time);

	std::ofstream m_file;

	bool m_isFirstFrame;
	Mat m_firstDepthFrame;
};

