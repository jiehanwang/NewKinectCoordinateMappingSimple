// NewKinectCoordinateMappingSimple.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include "opencv2\opencv.hpp"
using namespace cv;
using namespace std;

static const int        cDepthWidth  = 512;
static const int        cDepthHeight = 424;
static const int        cColorWidth  = 1920;
static const int        cColorHeight = 1080;

// Current Kinect
IKinectSensor*          m_pKinectSensor;
ICoordinateMapper*      m_pCoordinateMapper;
ColorSpacePoint*        m_pColorCoordinates;

// Frame reader
IMultiSourceFrameReader*m_pMultiSourceFrameReader;

RGBQUAD*                m_pColorRGBX;
RGBQUAD*               m_pOutputRGBX;


HRESULT InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout<<"No ready Kinect found!"<<endl;
		return E_FAIL;
	}

	return hr;
}

void ProcessFrame(INT64 nTime, 
				  const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, 
				  const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
				  const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight,
				  IplImage* img)
{
// 	if (m_hWnd)
// 	{
// 		if (!m_nStartTime)
// 		{
// 			m_nStartTime = nTime;
// 		}
// 
// 		double fps = 0.0;
// 
// 		LARGE_INTEGER qpcNow = {0};
// 		if (m_fFreq)
// 		{
// 			if (QueryPerformanceCounter(&qpcNow))
// 			{
// 				if (m_nLastCounter)
// 				{
// 					m_nFramesSinceUpdate++;
// 					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
// 				}
// 			}
// 		}
// 
// 		WCHAR szStatusMessage[64];
// 		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));
// 
// 		if (SetStatusMessage(szStatusMessage, 1000, false))
// 		{
// 			m_nLastCounter = qpcNow.QuadPart;
// 			m_nFramesSinceUpdate = 0;
// 		}
// 	}

	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pColorCoordinates && m_pOutputRGBX && 
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) && 
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
	{
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight,
			(UINT16*)pDepthBuffer,nDepthWidth * nDepthHeight, m_pColorCoordinates);
		if (SUCCEEDED(hr))
		{
			RGBQUAD c_green = {0, 255, 0}; 

			// loop over pixel of the output
			for (int depthIndex = 0; depthIndex < (nDepthWidth * nDepthHeight); ++depthIndex)
			{
				// default setting source to copy from the background pixel
				const RGBQUAD* pSrc = &c_green; 

				BYTE player = pBodyIndexBuffer[depthIndex];

				// if we're tracking a player for the current pixel, draw from the color camera
				if (player != 0xff)
				{
					// retrieve the depth to color mapping for the current depth pixel
					ColorSpacePoint colorPoint = m_pColorCoordinates[depthIndex];

					// make sure the depth pixel maps to a valid point in color space
					int colorX = (int)(floor(colorPoint.X + 0.5));
					int colorY = (int)(floor(colorPoint.Y + 0.5));
					if ((colorX >= 0) && (colorX < nColorWidth) && (colorY >= 0) && (colorY < nColorHeight))
					{
						// calculate index into color array
						int colorIndex = colorX + (colorY * nColorWidth);
						// set source for copy to the color pixel
						pSrc = m_pColorRGBX + colorIndex;
					}
				}

				// write output
				m_pOutputRGBX[depthIndex] = *pSrc;
			}

			// Draw the data with Direct2D
			//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
			for (int y=0;y<cDepthHeight;y++)
			{
				uchar* src_ptr = (uchar*)(img->imageData + y*img->widthStep);
				for (int x=0;x<cDepthWidth;x++)
				{
					(BYTE)src_ptr[3*x+0] = m_pOutputRGBX[y*cDepthWidth + x].rgbBlue;
					(BYTE)src_ptr[3*x+1] = m_pOutputRGBX[y*cDepthWidth + x].rgbGreen;
					(BYTE)src_ptr[3*x+2] = m_pOutputRGBX[y*cDepthWidth + x].rgbRed;

				}
			}
		}
	}
}

void Update(IplImage* img)
{
	if (!m_pMultiSourceFrameReader)
	{
		return;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}

		SafeRelease(pBodyIndexFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		INT64 nDepthTime = 0;
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int nBodyIndexWidth = 0;
		int nBodyIndexHeight = 0;
		UINT nBodyIndexBufferSize = 0;
		BYTE *pBodyIndexBuffer = NULL;

		// get depth frame data

		hr = pDepthFrame->get_RelativeTime(&nDepthTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		// get body index frame data

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);            
		}

		if (SUCCEEDED(hr))
		{
			ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight, 
				pColorBuffer, nColorWidth, nColorHeight,
				pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight,
				img);
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pBodyIndexFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
}

int _tmain(int argc, _TCHAR* argv[])
{
	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	// create heap storage for composite image pixel data in RGBX format
	m_pOutputRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	// create heap storage for the coorinate mapping from depth to color
	m_pColorCoordinates = new ColorSpacePoint[cDepthWidth * cDepthHeight];

	InitializeDefaultSensor();

	IplImage* showImg_color = cvCreateImage(cvSize(cDepthWidth,cDepthHeight),8,3);
	cvNamedWindow("show_color",1);

	while (1)
	{
		Update(showImg_color);
		cvShowImage("show_color",showImg_color);
		waitKey(10);
	}




	//Release
	cvDestroyWindow("show_color");
	cvReleaseImage(&showImg_color);

	
	if (m_pOutputRGBX)
	{
		delete [] m_pOutputRGBX;
		m_pOutputRGBX = NULL;
	}


	if (m_pColorRGBX)
	{
		delete [] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pColorCoordinates)
	{
		delete [] m_pColorCoordinates;
		m_pColorCoordinates = NULL;
	}


	// done with frame reader
	SafeRelease(m_pMultiSourceFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
	return 0;
}

