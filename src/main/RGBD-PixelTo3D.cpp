// RGBD-PixelTo3D.cpp: define o ponto de entrada para o aplicativo.
//

#include "RGBD-PixelTo3D.hpp"
#include "RGBD-Pose.hpp"

#include <iostream>

using Eigen::Matrix3d;
using RGBD::CameraIntrinsics;
using RGBD::PoseCoords;
using RGBD::DepthPixel;

/// <summary>
/// Creates PixelTo3D object, used for converting rgb and depth pixel
/// (or images) to 3D.
/// </summary>
/// <param name="intrinsics">Camera intrinsic parameters</param>
PixelTo3D::PixelTo3D(CameraIntrinsics intrinsics)
{
	// (fx s x0)
	// (0 fy x1)
	// (0  0  1)
	Matrix3d intrinsicsMatrix;
	intrinsicsMatrix(0, 0) = intrinsics.focalX;
	intrinsicsMatrix(0, 1) = intrinsics.skew;
	intrinsicsMatrix(0, 2) = intrinsics.offsetX;
	intrinsicsMatrix(1, 0) = 0;
	intrinsicsMatrix(1, 1) = intrinsics.focalY;
	intrinsicsMatrix(1, 2) = intrinsics.offsetY;
	intrinsicsMatrix(2, 0) = 0;
	intrinsicsMatrix(2, 1) = 0;
	intrinsicsMatrix(2, 2) = 1;

	inverseIntrisicMatrix = intrinsicsMatrix.inverse();
}

PoseCoords* PixelTo3D::convertTo3D(int pixelX, int pixelY, double depth)
{
	Eigen::Vector3d worldCoords(pixelX, pixelY, 1);
	worldCoords = inverseIntrisicMatrix * worldCoords;
	worldCoords *= depth;
	return new PoseCoords(worldCoords.x(), worldCoords.y(), worldCoords.z());
}

PoseCoords* PixelTo3D::convertTo3D(DepthPixel* depthImage, int pixelX, int pixelY, int h, int w, int k)
{
	if (pixelX > w) [[unlikely]] {
		std::cerr << "Pixel x coordinate is bigger than screen width\n";
		return new PoseCoords(0, 0, 0);
	}
	if (pixelY > h) [[unlikely]] {
		std::cerr << "Pixel y coordinate is bigger than screen heigth\n";
		return new PoseCoords(0, 0, 0);
	}
	if (pixelX < 0 || pixelY < 0) [[unlikely]] {
		std::cerr << "Wrong pixel coordinates\n";
		return new PoseCoords(0, 0, 0);
	}

	// knn
	double bestDepth = std::numeric_limits<double>::max();

	const int xLessK = pixelX - k;
	const int minX = (xLessK >= 0) ? xLessK : 0;
	const int xPlusK = pixelX + k;
	const int maxX = (xPlusK <= w) ? xPlusK : w;
	const int yLessK = pixelY - k;
	const int minY = (yLessK >= 0) ? yLessK : 0;
	const int yPlusK = pixelY + k;
	const int maxY = (yPlusK <= h) ? yPlusK : h;

	for (int j = minX; j <= maxX; j++) {
		for (int k = minY; k <= maxY; k++) {
			const int index = calcDepthIndex(j, k, w);
			if (index < 0 || index > w * h) [[unlikely]]
				continue;

			const double depth = depthImage[index];
			if (depth == 0) continue;

			if (depth < bestDepth) {
				bestDepth = depth;
			}
		}
	}

	[[unlikely]] bestDepth = bestDepth == std::numeric_limits<double>::max() ? 0 : bestDepth;

	return convertTo3D(pixelX, pixelY, bestDepth);
}

constexpr int PixelTo3D::calcDepthIndex(int x, int y, int width) {
	return ((y * width) + x);
}