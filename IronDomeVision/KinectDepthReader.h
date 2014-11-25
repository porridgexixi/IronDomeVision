/*******************************************************************
Author: Wanxi Liu
Date: Nov 23, 2014


*******************************************************************/
#pragma once

#include <Kinect.h>
#include <opencv2\opencv.hpp>

class CKinectDepthReader
{
public:
	CKinectDepthReader();
	~CKinectDepthReader();

	// Initialize default Kinect sensor
	HRESULT                 InitializeDefaultSensor();

	void					UpdateDepthMat(cv::Mat a_depthMat, double& a_depthTime);

	int m_width;
	int m_height;

	// Coordinate Mapper
	ICoordinateMapper*		m_pCoordinateMapper;

private:
	// Current Kinect
	IKinectSensor*          m_pKinectSensor;

	// Depth reader
	IDepthFrameReader*      m_pDepthFrameReader;

	// Description
	IFrameDescription*		m_pDescription;

	UINT m_bufferSize;

};

