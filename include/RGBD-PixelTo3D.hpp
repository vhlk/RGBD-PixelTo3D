#pragma once

#include <tuple>
#include <Eigen/Dense>
#include "AuxImageType.hpp"

namespace RGBD {
	/* Camera intrisic parameters */
	struct CameraIntrinsics {
		double focalX;
		double focalY;
		double offsetX;
		double offsetY;
		double skew;

		CameraIntrinsics(double focalX, double focalY, double offsetX, double offsetY, double skew = 0) {
			this->focalX = focalX;
			this->focalY = focalY;
			this->offsetX = offsetX;
			this->offsetY = offsetY;
			this->skew = skew;
		}
	};

	struct PoseCoords {
		double x;
		double y;
		double z;

		PoseCoords(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}

		std::string to_string() const {
			return "PoseCoords: " + '(' + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
		}

		friend std::ostream& operator<<(std::ostream& os, const PoseCoords& dt);
		friend std::ostream& operator<<(std::ostream& os, const PoseCoords* dt);
	};

	inline std::ostream& operator<<(std::ostream& os, const PoseCoords& pc) {
		os << "PoseCoords: " << '(' << pc.x << ", " << pc.y << ", " << pc.z << ')';
		return os;
	}

	inline std::ostream& operator<<(std::ostream& os, const PoseCoords* pc) {
		os << "PoseCoords: " << '(' << pc->x << ", " << pc->y << ", " << pc->z << ')';
		return os;
	}
}

/// <summary>
/// Converts a pixel (2D coordinate) to world coordinates (3D)
/// using RGBD camera data.
/// Pixels coordinates starts at top left.
/// </summary>
class PixelTo3D {
public:
	PixelTo3D(RGBD::CameraIntrinsics intrinsics);
	
	/// <summary>
	/// Converts a pixel to 3D using camera intrinsics and depth. The object is created with new, so its the user job to delete after usage.
	/// </summary>
	/// <param name="pixelX">The x coordinate of the pixel</param>
	/// <param name="pixelY">The y coordinate of the pixel</param>
	/// <param name="depth">The z coordinate (from the camera depth)</param>
	/// <returns>
	/// PoseCoords: coordinates in 3D
	/// </returns>
	RGBD::PoseCoords* convertTo3D(int pixelX, int pixelY, double depth);

	/// <summary>
	/// Converts a pixel to 3D using its depth and rgb image.
	/// kNN algoritm can be used to calculate the best (closer) depth
	/// for the ROI, ignoring 0 as it means it could not be estimated. Disabled by default.
	/// The object is created with new, so its the user job to delete after usage.
	/// </summary>
	/// <param name="depthImage">Depth Image</param>
	/// <param name="pixelX">The pixel x coordinate</param>
	/// <param name="pixelY">The pixel y coordinate</param>
	/// <param name="h">Height</param>
	/// <param name="w">Width</param>
	/// <param name="knn">Number of pixels for each direction.</param>
	/// <returns></returns>
	RGBD::PoseCoords* convertTo3D(RGBD::DepthPixel* depthImage, int pixelX, int pixelY, int h, int w, int knn = 0);

private:
	Eigen::Matrix3d inverseIntrisicMatrix;

	constexpr int calcDepthIndex(int x, int y, int width); // depth has only 1 dimension
};