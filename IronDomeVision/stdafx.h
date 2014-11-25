// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#include "targetver.h"

#include <zmq.hpp>

#include <stdio.h>
#include <tchar.h>
#include <ctime>
#include <string>

#include <iostream>
#include <iomanip>

#include <Kinect.h>
#include <opencv2\opencv.hpp>


#include "KinectDepthReader.h"
#include "BallTracking.h"

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
