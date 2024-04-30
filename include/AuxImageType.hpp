#pragma once

#include <cstdint>

namespace RGBD {
	/// <summary>
	/// Holds the value of a single color image pixel in 24-bit RGB format.
	/// Based on OpenNI, so it can be compatible with OpenCV but not needing its depedencies.
	/// Can be converted to OpenCV Mat with simple cast: <para />
	/// cv::Mat mat(h, w, CV_8UC3); <para />
	/// mat.data = (uchar*) rgb888Pixel; <para />
	/// For more info, check https://abre.ai/rgb888pixelbyteorder.
	/// </summary>
	typedef struct
	{
		/* Red value of this pixel. */
		uint8_t r;
		/* Green value of this pixel. */
		uint8_t g;
		/* Blue value of this pixel. */
		uint8_t b;
	} RGB888Pixel;

	/// <summary>
	/// Pixel type used to store depth images.
	/// Based on OpenNI, so it can be compatible with OpenCV but not needing its depedencies.
	/// Can be converted to OpenCV Mat with simple cast: <para />
	/// cv::Mat mat(h, w, CV_16UC1); <para />
	/// mat.data = (uchar*) depthPixel; <para />
	/// For more info, check https://abre.ai/rgb888pixelbyteorder.
	/// </summary>
	typedef uint16_t DepthPixel;
}