/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <OpenNI.h>

//#include "OpenNI/Samples/Common/OniSampleUtilities.h"

using namespace openni;

int main()
{
	OpenNI::initialize();

	Device device;
	device.open(ANY_DEVICE);
	device.setDepthColorSyncEnabled(true);
	device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);




	VideoStream depth;
	depth.create(device, SENSOR_DEPTH);


	VideoStream color;
	color.create(device, SENSOR_COLOR);


	depth.start();
	color.start();

	VideoFrameRef framed;
	VideoFrameRef framec;

	for (long long i = 0; i < 100000; i++)
	{
		depth.readFrame(&framed);
		color.readFrame(&framec);


		DepthPixel* pDepth = (DepthPixel*)framed.getData();
		RGB888Pixel * pColor = (RGB888Pixel*)framec.getData();

		int middleIndex = (framed.getHeight()+1)*framed.getWidth()/2;

		printf("[%08llu] %8d\n", (long long)framec.getTimestamp(), pDepth[middleIndex]);
		printf("[%08llu] %8d %8d %8d\n", (long long)framed.getTimestamp(),
											pColor[middleIndex].r,
											pColor[middleIndex].g,
											pColor[middleIndex].b);

	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
