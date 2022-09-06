---
title:  "Capturing data from Kinect v2 using OpenCV"
author: matheus
date:   2022-08-15 10:00:00 -0300
categories: [3D Reconstruction, LiDAR ]
tags: [kinect, opencv, pcl, openni, 3d]
pin: false
---

# What is Kinect?

Kinect is a movement sensor developed for use on Xbox 360 and Xbox One by Prime Sense in a collaboration with Microsoft. The device contains microphones, RGB cameras, infrared emissors and a monocromátic CMOS sensor for sampling infrared. By calibrating the infrared emissor and CMOS sensor it is possible to obtain depth maps and reconstruct the scene.

The device was created as a natural interface for users to interact with video games without the need for physical controllers. It is capable to detect pose of up to 4 people simultaneously.

In this article are the steps necessary to access Kinect data using OpenCV on Linux.

# Materials

- [Kinect v2](https://produto.mercadolivre.com.br/MLB-2031065410-sensor-kinect-microsoft-xbox-one-original-novo-vitrine-nf-_JM#position=3&search_layout=grid&type=item&tracking_id=30704690-2e9a-4958-a95e-fb457fdb946a)
- [Kinect v2 Adapter](https://produto.mercadolivre.com.br/MLB-1895436597-adaptador-kinect-30-conector-xbox-one-x-one-s-windows-10-_JM)
- PC with linux

# Dependencies

During the wrinting of this article, two operational systems were used: Ubuntu 20.04 and Ubuntu 22.04 and because of that it will be shown the APT commands used to install pre-compiled libraries. For users of other OS it will be show compilation steps for the dependencies.

## OpenNI

OpenNI (Open Natural Interaction) is an Open Source project that aims to improve standardization of natural user interfaces. It provides an API that encapsulates hardware details and facilitates integration with applications.

This library was mainly maintained by PrimeSense, but its original repository was discontinued when the company was bought by Apple in 2013. Since then old partners and employees from PrimeSense made a fork from OpenNI 2 which is currently maintained by StructureIO.

- Repo: [https://github.com/structureio/OpenNI2](https://github.com/structureio/OpenNI2)
- APT Commmand: ```apt install libopenni2-dev openni2-utils```

## OpenCV

OpenCV (Open Computer Vision) is an Open Source library that reunites computer vision algorithms on different areas: image processing, 3D reconstruction, optimization, clustering, tracking, descriptors, matching, object recognition and deep learning. It has support for multiple programming languages like: C++, python, java e MATLAB.

 - Repo: [https://github.com/opencv/opencv](https://github.com/opencv/opencv)
 - APT Commmand: ```apt install libopencv-dev```


The following steps are necessary to compile a version of OpenCV with support to OpenNI2:

```bash
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 4.6.0
cd ../opencv
git checkout 4.6.0
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PERF_TESTS=OFF -DOPENCV_ENABLE_NONFREE=OFF -DOPENCV_EXTRA_MODULES=../opencv_contrib/modules -DWITH_OPENNI2=ON ..
make -j 4
sudo make install
```

## PCL

PCL (Point Clout Library) is a C++ library specialized in processing point clouds. It provides implementations of filters, key point selection, surface reconstruction, sensor communication and visualization.

 - Repositório: [https://github.com/PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)
 - Comando APT: ```apt install libpcl-dev```

# Making it work

The libraries in the last session allow high level programming that provides functions to recover and reconstruct 3D data. However, to make them work we firstly need to compile a driver that manages communication between OS and Kinect. To do so it is possible to utilize the project libfreenect, an open source Kinect driver developed mostly for linux but capable of being compiled on windows.

As the main focus of this article is to utilize the Kinect version 2, on this session the steps necessary to compile and configure the [LibFreenect2](https://github.com/OpenKinect/libfreenect2) are shown.

## Compiling libfreenect2

```bash
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
git checkout v0.2.1
mkdir build
cd build
cmake -DBUILD_OPENNI2_DRIVER=ON -DCMAKE_BUILD_TYPE=Release ..
make -j 4
sudo make install
sudo make install-openni2
```

## Allow access to device

After compilation and installation of the project, it is necessary to grant users access to Kinect device, otherwise root permission will be needed to capture data from the sensor. It can be done by executing the following commands:

```bash
cd libfreenect2
sudo cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```

**It is necessary to disconnect the Kinect sensor from the USB port and reconnect again for changes to take place.**

## Testing with NiViewer2

After following the steps above the command NiViewer2 can be used to check if the environment is correct configurated:

```bash
sudo apt install openni2-utils # caso não tenha sido instalado ainda
NiViewer2
```

The execution of the above command is expected to look like the following image:

![NiViewer Example]({{site.baseurl}}/kinect_v2_com_opencv/niviewer2.png "Tela NiViewer")


```
[Info] [Freenect2Impl] enumerating devices...
[Info] [Freenect2Impl] 10 usb devices connected
[Info] [Freenect2Impl] found valid Kinect v2 @2:2 with serial 067886733447
[Info] [Freenect2Impl] found 1 devices
[Info] [Freenect2DeviceImpl] opening...
[Info] [Freenect2DeviceImpl] transfer pool sizes rgb: 20*16384 ir: 60*8*33792
[Info] [Freenect2DeviceImpl] opened
[Info] [Freenect2DeviceImpl] starting...
[Info] [Freenect2DeviceImpl] submitting rgb transfers...
[Info] [Freenect2DeviceImpl] submitting depth transfers...
[Info] [Freenect2DeviceImpl] started
[Info] [DepthPacketStreamParser] 32 packets were lost
[Info] [DepthPacketStreamParser] 13 packets were lost
[Info] [TurboJpegRgbPacketProcessor] avg. time: 9.53263ms -> ~104.903Hz
```

# Program Sample

If the environment is correct configured, on this section is provided a sample code using OpenCV to capture data from sensor and PCL to visualize the point cloud. After compiling and executing the program it is expected a view like the video of the last session.

## Source Code


```cmake
find_package(OpenCV 4 COMPONENTS core highgui videoio imgcodecs)

if(${OpenCV_FOUND})
else()
        find_package(OpenCV 2 COMPONENTS core highgui REQUIRED)
endif()


find_package(PCL COMPONENTS visualization REQUIRED)

set(OPENNI_CAPTURE_INCLUDE_DIRS 
	${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB OPENNI_CAPTURE_SOURCES main.cpp)
include_directories(${OPENNI_CAPTURE_INCLUDE_DIRS})

add_executable(openni-capture ${OPENNI_CAPTURE_SOURCES})
target_link_libraries(openni-capture ${OpenCV_LIBS} ${PCL_LIBRARIES})
```

```c++
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>

cv::VideoCapture cap(1, cv::CAP_OPENNI2); // Configures cv::VideoCapture to open device 1 using OpenNI2

void getPointCloud(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.removeAllPointClouds(); // Cleans previous point cloud on view

    cv::Mat pointCloudFrame, frame;

    cap.grab(); // Grabs a sensor reading
    cap.retrieve(pointCloudFrame, cv::CAP_OPENNI_POINT_CLOUD_MAP); // Recovers point cloud on matrix format from sensor data
    cap.retrieve(frame, cv::CAP_OPENNI_BGR_IMAGE); // Obtains color information aligned with point cloud

    pcl::visualization::CloudViewer::ColorCloud* cloud = new pcl::visualization::CloudViewer::ColorCloud();

    for(int i = 0; i<pointCloudFrame.rows; i++) // For each point on the matrix
    {
        for(int j = 0; j<pointCloudFrame.cols; j++) // For each point on the matrix
        {
            cv::Vec3b color = frame.at<cv::Vec3b>(i,j); // Gets point color 
            pcl::PointXYZRGB point(color[2], color[1], color[0]); // Creates a PCL point containing color

            cv::Vec3f coords = pointCloudFrame.at<cv::Vec3f>(i,j); // Gets X,Y,Z position of point
            point.x = coords[0]; // Sets coodinate on point
            point.y = coords[1]; // Sets coodinate on point
            point.z = coords[2]; // Sets coodinate on point

            cloud->push_back(point); // Adds point on point cloud
        }
    }

    viewer.addPointCloud(pcl::visualization::CloudViewer::ColorCloud::ConstPtr(cloud)); // Sends a point cloud to be rendered on screen
}


int main(int argc, char** argv)
{

    if(!cap.isOpened())
       return -1;

    cap.set(cv::CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1); // Configures OpenNI to generate point clouds with color information

    pcl::visualization::CloudViewer viewer("point cloud"); // Initializes a Point Cloud Viewer interface
    viewer.runOnVisualizationThread(getPointCloud); // Defines callback function to render scene

    while(!viewer.wasStopped()); // Waits for window closure

    cap.release(); // Closes communication with device
    
    return 0;

}
```

## Result
<div style="position:relative;width:100%;padding-bottom:56.25%">
<iframe style="position:absolute;top:0;left:0;width:100%;height:100%;border:0" src="https://www.youtube.com/embed/199c-im-FOM" allowfullscreen></iframe>
</div>

<div>
<h3>
<p>In case of any doubts or ideas send me a <a href="mailto:matheusbarcelosoliveira@gmail.com">email</a>,</p>
<p><a href="https://github.com/mathbarc">Matheus Barcelos de Oliveira</a><br/>
Computer Engineer</p>
</h3>
</div>
