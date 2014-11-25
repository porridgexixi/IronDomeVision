#include "stdafx.h"
#include "KinectDepthReader.h"
#include <iostream>


CKinectDepthReader::CKinectDepthReader() :
m_pKinectSensor(NULL),
m_pDepthFrameReader(NULL)
{}


CKinectDepthReader::~CKinectDepthReader()
{
	// done with depth frame reader
	SafeRelease(m_pDepthFrameReader);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

HRESULT CKinectDepthReader::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		// Sensor
		hr = m_pKinectSensor->Open();


		//Source
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		//Reader
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->get_FrameDescription(&m_pDescription);
			m_pDescription->get_Width(&m_width);
			m_pDescription->get_Height(&m_height);
			m_bufferSize = m_width * m_height * sizeof(unsigned short);
		}

		if (SUCCEEDED(hr))
		{
			m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cerr << "No ready Kinect found!" << std::endl;
		return E_FAIL;
	}

	return hr;
}

void CKinectDepthReader::UpdateDepthMat(cv::Mat a_depthMat, double& a_depthTime)
{
	if (!m_pDepthFrameReader)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;
	cv::Mat bufferMat(m_height, m_width, CV_16UC1);

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&m_bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));

			if (SUCCEEDED(hr))
			{
				//std::cout << "successfully grab depth frame" << std::endl;
				//bufferMat.convertTo(a_depthMat, CV_32F, (-1.0f), 0.0f);
				bufferMat.copyTo(a_depthMat);

				INT64 nTime;
				pDepthFrame->get_RelativeTime(&nTime);
				a_depthTime = nTime / 10000000.0;
			}
		}
	}

	SafeRelease(pDepthFrame);
}