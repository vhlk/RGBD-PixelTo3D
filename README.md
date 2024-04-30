# RGB-D Pose Tracking

# Table of Contents
1. [Using RGB-D Pose Tracking](#using)
2. [Installation](#installation)

## Using RGB-D Pose Tracking <a name="using"></a>
First you need to get the sensor intrinsic parameters.
Then create a object for storing the values:

```
// example values, get them from your chosen sensor
RGBD::CameraIntrinsics camIntrisics(/*focal x*/ 1, /*focal y*/ 2,
		/*offset x*/ 3, /*offset x*/ 4, /*(optional) skew*/ -0.1);
```

Start pose:
```
Pose p(camIntrisics, img_width, img_height);
```

We assume the rgb and depth image are char arrays.
We can get the person coordinates with:
```
const auto optCoordsUpd = p.update((RGBD::RGB888Pixel*)rgbImage,
		(RGBD::DepthPixel*)depthImage);
```

PS: We created those objects so we can store the data the same way as OpenCV, but without the dependency.
Given a cv::Mat storing the images, we could have the code above as:
```
const auto optCoords = p.update((RGBD::RGB888Pixel*)rgbImage.data,
		(RGBD::DepthPixel*)depthImage.data);
```

Other functions are provided:
* RGBDandMediaPipeUpdate, which also returns 3D coordinates from MediaPipe;
* update2Dand3D, which also returns pixel coordinates, normalized between 0 and 1 (check https://google.github.io/mediapipe/solutions/pose.html for more info).

PixelTo3D class is also available, which can convert pixel to 3D coordinates. It provides the following functions:
* convertTo3D(int pixelX, int pixelY, double depth);
* convertTo3D(RGBD::DepthPixel* depthImage, int pixelX, int pixelY, int h, int w, int knn = 0);

## Installation <a name="installation"></a>

### Installation C++
1. Install python requirements:
```
pip install -r requirements.txt
```

2. Build with cmake (or cmake-gui).

### Python Wrapper
You can easily install with (at project root):
```
pip install .
```
