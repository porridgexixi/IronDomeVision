#include "stdafx.h"
#include "BallTracking.h"


#define INTERVAL 1.2
#define INTERVAL_BUF 1.0
#define MIN_DISTANCE 1.2

CBallTracking::CBallTracking()
{
	// Initial value for SimpleBlobDetector Parameters
	m_params.filterByArea = true;
	m_params.filterByCircularity = false;
	m_params.filterByColor = false;
	m_params.maxThreshold = 250;
	m_params.minThreshold = 10;
	m_params.minDistBetweenBlobs = 20;
	m_params.thresholdStep = 15;
	m_params.maxArea = 100000;
	m_params.minArea = 65;
	m_params.minCircularity = 0.51;
	m_params.maxCircularity = 1.0;

	// Create SimpleBlobDetector
	SimpleBlobDetector blobDetector(m_params);
	m_blobDetector = blobDetector;
	m_blobDetector.create("SimpleBlob");

	m_id = 0;
	m_lastDetectTime = 0;
	m_lastBallsCount = 0;

	m_isFirstFrame = true;

	m_file.open("data.txt");
}


CBallTracking::~CBallTracking()
{
	m_file.close();
}

void CBallTracking::UpdateResults(Mat a_depthMat, double a_depthTime, ICoordinateMapper* a_mapper, bool a_drawResults)
{
	if (!m_keyPoints.empty())
	{
		m_keyPoints.clear();
	}

	if (!m_balls.empty())
	{
		m_balls.clear();
	}

	static Mat depthMatF(a_depthMat.size(), CV_32FC1);
	static Mat depthImg(a_depthMat.size(), CV_8UC1);
	static Mat depthImgRGB(a_depthMat.size(), CV_8UC3);

	// thresholding
	a_depthMat.convertTo(depthMatF, CV_32F, 1.0, 0.0);
	threshold(depthMatF, depthMatF, 3500, 0, CV_THRESH_TOZERO_INV);
	threshold(depthMatF, depthMatF, 500, 0, THRESH_TOZERO);

	// blob tracking
	depthMatF.convertTo(depthImg, CV_8U, -255.0f / 3500.0f, 255.0f);

	if (a_depthTime < 1)
	{
		return;
	}

	if (m_isFirstFrame)
	{
		m_firstDepthFrame = depthImg.clone();
		m_isFirstFrame = false;
	}

	//subtract(depthImg, m_firstDepthFrame, depthImg);
	Mat subBg(depthImg.size(), CV_8UC1);
	absdiff(depthImg, m_firstDepthFrame, subBg);
	threshold(subBg, subBg, 20, 255, CV_THRESH_BINARY);

	m_blobDetector.detect(depthImg, m_keyPoints, subBg);
	//Mat maskDepthImg;
	//depthImg.copyTo(maskDepthImg, subBg);
	//imshow("mask", maskDepthImg);

	CameraSpacePoint* cameraPoints = new CameraSpacePoint[10];
	int ballCount = GetCameraSpaceCoordinate(a_depthMat, a_mapper, cameraPoints);


	if (ballCount != 0)
	{
		GetFinalResults(a_depthTime);
	}

	UpdateBufferBalls(a_depthTime);
	//std::cout << "buffer size: " << m_bufferBalls.size() << std::endl;

	if (a_drawResults)
	{
		cvtColor(depthImg, depthImgRGB, CV_GRAY2RGB);
		DrawResults(depthImgRGB);
	}

	delete[] cameraPoints;

}

int CBallTracking::GetCameraSpaceCoordinate(Mat a_depthMat, ICoordinateMapper* a_mapper, CameraSpacePoint* a_cameraPoints)
{
	// wrap and send result
	DepthSpacePoint* depthPoints = new DepthSpacePoint[10];
	UINT16* depth = new UINT16[10];

	float* radius = new float[10];

	int i = 0;
	for (std::vector<KeyPoint>::iterator it = m_keyPoints.begin(); it != m_keyPoints.end(); it++)
	{
		//std::cout << "key point num: " << m_keyPoints.size() << std::endl;

		depthPoints[i].X = round(it->pt.x);
		depthPoints[i].Y = round(it->pt.y);

		radius[i] = it->size;
	
		depth[i] = a_depthMat.at<UINT16>(Point(round(it->pt.x), round(it->pt.y)));

		//std::cout << "i=" << i << " depthPoint " << depthPoints[i].X  <<" "<<depthPoints[i].Y << " depth " << depth[i] << std::endl;

		i++;
	}

	UINT depthCount = m_keyPoints.size();
	if (depthCount != 0)
	{
		a_mapper->MapDepthPointsToCameraSpace(depthCount, depthPoints, depthCount, depth, depthCount, a_cameraPoints);
	}

	UINT ballCount = 0;
	if (!m_cameraPoints.empty())
	{
		m_cameraPoints.clear();
	}
	for (int i = 0; i < depthCount; i++)
	{
		float x = depth[i] / 1000.0;
		float desiredRadius = -0.2661 *x*x*x + 3.7112 *x*x - 15.842 *x + 26.789;
		
		if (abs(radius[i] - desiredRadius) / desiredRadius < 0.5)
		{
			//std::cout << "accuracy" << abs(radius[i] - desiredRadius) / desiredRadius << std::endl;
			m_cameraPoints.push_back(a_cameraPoints[i]);
			ballCount++;
		}
	}

	delete[] depthPoints;
	delete[] depth;
	return ballCount;
}


void CBallTracking::GetFinalResults(double a_depthTime)
{
	std::vector<ballData> newBalls;
	for (int i = 0; i < m_cameraPoints.size(); i++)
	{
		ballData tempBall;
		tempBall.x = m_cameraPoints[i].X;
		tempBall.y = m_cameraPoints[i].Y;
		tempBall.z = m_cameraPoints[i].Z;
		tempBall.timestamp = a_depthTime;
		tempBall.id = -1;		//Unassigned

		newBalls.push_back(tempBall);
	}

	if (m_lastDetectTime == 0)
	{
		m_lastDetectTime = a_depthTime;
	}

	if (a_depthTime - m_lastDetectTime > INTERVAL)
	{
		m_id++;
		for (int i = 0; i < newBalls.size(); i++)
		{
			newBalls[i].id = m_id + i;
		}
	}
	else
	{
		if (m_lastBallsCount == 0)
		{
			m_id++;
			for (int i = 0; i < newBalls.size(); i++)
			{
				newBalls[i].id = m_id + i;
			}
		}
		else
		{
			for (int j = 0; j < newBalls.size(); j++)
			{
				Point3d currBall(newBalls[j].x, newBalls[j].y, newBalls[j].z);
				bool sameBall = false;
				double min_timestamp = 0;
				double max_distance = MIN_DISTANCE;

				for (int i = 0; i < m_bufferBalls.size(); i++)
				{
					Point3d lastBall(m_bufferBalls[i].x, m_bufferBalls[i].y, m_bufferBalls[i].z);
					double dist = norm(currBall - lastBall);
					if (dist < 2 * MIN_DISTANCE)
					{
						//std::cout << "norm:" << norm(currBall - lastBall) << std::endl;
						
						if (m_bufferBalls[i].timestamp > min_timestamp && dist < max_distance)
						{
							newBalls[j].id = m_bufferBalls[i].id;
							min_timestamp = m_bufferBalls[i].timestamp;
							max_distance = dist;
							sameBall = true;
						}
					}
				}
				if (!sameBall)
				{
					m_id++;
					newBalls[j].id = m_id;
				}
			}
			for (int j = 0; j < newBalls.size(); j++)
			{
				if (newBalls[j].id == -1)
				{
					m_id++;
					newBalls[j].id = m_id;
				}
			}
		}

	}

	m_balls = newBalls;
	m_lastDetectTime = a_depthTime;
	m_lastBallsCount = m_cameraPoints.size();
}

void CBallTracking::UpdateBufferBalls(double a_time)
{
	if (!m_bufferBalls.empty())
	{
		for (std::vector<ballData>::iterator it = m_bufferBalls.begin(); it != m_bufferBalls.end(); it++)
		{
			if (a_time - (*it).timestamp > INTERVAL_BUF)
			{
				//std::cout << "erase!" << std::endl;
				m_bufferBalls.erase(it);
				it--;
			}
			else
			{
				break;
			}
		}
	}

	for (std::vector<ballData>::iterator it1 = m_balls.begin(); it1 != m_balls.end(); it1++)
	{
		Point3d currBall(it1->x, it1->y, it1->z);
		for (std::vector<ballData>::iterator it2 = m_bufferBalls.begin(); it2 != m_bufferBalls.end(); it2++)
		{
			Point3d buffBall(it2->x, it2->y, it2->z);
			if (norm(currBall - buffBall) < MIN_DISTANCE)
			{
				m_bufferBalls.erase(it2);
				it2--;
				continue;
			}
		}
		m_bufferBalls.push_back(*it1);
	}
}

void CBallTracking::DrawResults(Mat a_depthImg)
{
	if (m_keyPoints.empty())
	{
		imshow("Blob Tracking", a_depthImg);
		return;
	}

	int i = 0;
	for (std::vector<KeyPoint>::iterator it = m_keyPoints.begin(); it != m_keyPoints.end(); it++)
	{
		int ptx = it->pt.x;
		int pty = it->pt.y;
		float radius = it->size;

		//std::cout << "Ball tracking results:" << ptx << " " << pty << " " << round(radius) << " " << std::endl;
		circle(a_depthImg, Point(ptx, pty), round(radius), Scalar(0, 0, 255), 2);

		//Display information on the image
		std::stringstream blobInfo;
		blobInfo.setf(std::ios::fixed);

		/*
		blobInfo << " ("
			<< m_balls[i].x << " " << m_balls[i].y << " " << m_balls[i].z
			<<")Time: " << m_balls[i].timestamp;
		*/
			

		//blobInfo << radius;

		string blobInfoStr = blobInfo.str();
		putText(a_depthImg, blobInfoStr, Point(ptx - 10, pty - 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 240));

		//std::cout << blobInfoStr << std::endl;
		//m_file << blobInfoStr << std::endl;

		i++;
	}

	imshow("Blob Tracking", a_depthImg);
}