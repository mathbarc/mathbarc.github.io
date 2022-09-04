---
title:  "Capturing data from Kinect v2 using OpenCV"
author: matheus
date:   2022-08-15 10:00:00 -0300
categories: [3D Reconstruction, LiDAR ]
tags: [kinect, opencv, pcl, openni, 3d]
pin: false
---

# What is Kinect?

Kinect is a moviment sensor develop for use on Xbox 360 and Xbox One by Prime Sense in a colaboration with Microsoft. The device contains microphones, RGB cameras, infrared emissors and a monocromátic CMOS sensor for sampling infrared. By calibrating the infrared emissor and CMOS sensor it is possible to obtain depth maps and reconstruct the scene.

The device was created as a natural interface for users to interact with video games without the need for fisical controllers. It is capable to detect pose of up to 4 people simultaneously.

In this article are the steps necessary to access Kinect data using OpenCV on Linux.

# Materials

- [Kinect v2](https://produto.mercadolivre.com.br/MLB-2031065410-sensor-kinect-microsoft-xbox-one-original-novo-vitrine-nf-_JM#position=3&search_layout=grid&type=item&tracking_id=30704690-2e9a-4958-a95e-fb457fdb946a)
- [Kinect v2 Adapter](https://produto.mercadolivre.com.br/MLB-1895436597-adaptador-kinect-30-conector-xbox-one-x-one-s-windows-10-_JM)
- PC with linux

# Dependencies

During wrinting of this article, two operational systems were used: Ubuntu 20.04 and Ubuntu 22.04 and because of that it will be shown the APT commands used to install pre-compiled libraries. For users of other OS it will be show compilation steps for the dependencies.

## OpenNI

OpenNI (Open Natural Interaction) is a Open Source project that aims to improve standarization of natural user interfaces. It provides a API that incapsulates hardware details and facilitates integration with applications.

This library was mainly mantained by PrimeSense, but it's original repository was discontinued when the company was bought by Apple in 2013. Since then old partners and employees from PrimeSense made a fork from OpenNI 2 which is currently mantained by StructureIO.

- Repo: [https://github.com/structureio/OpenNI2](https://github.com/structureio/OpenNI2)
- APT Commmand: ```apt install libopenni2-dev openni2-utils```

## OpenCV

OpenCV (Open Computer Vision) is a Open Source library that reunites computer vision algorithms on diferent areas: image processing, 3D reconstruction, optimization, clustering, tracking, descriptors, matching, object recognition and deep learning. It has support for multiple programming languages like: C++, python, java e MATLAB.

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

A PCL (Point Clout Library) é uma biblioteca escrita em C++ especializada no processamento de núvens de pontos. Esta biblioteca possuí algoritmos para filtro, seleção de pontos chave, reconstrução de superfícies, comunicação com sensores, visualização de núvens de pontos e outros tópicos.

 - Repositório: [https://github.com/PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)
 - Comando APT: ```apt install libpcl-dev```

# Making it work

As bibliotecas descritas na sessão anterior serão nossas interfaces de programação em um nível mais elevado, para que possamos extrair os dados do sensor primeiro é necessário um driver que gerenciará a comunicação entre o sistema operacional e kinect. Como os drivers oficiais do kinect são proprietários e apenas disponíveis na Kinect SDK para o windows, a comunidade inicio a criação de um drive para o linux: o freenect, desenvolvido para a primeira versão do kinect, e freenect2, desenvolvido para a segunda versão do kinect.

Neste artigo concentraremos no freenect2 pois estamos trabalhando com o Kinect v2. Nas seguintes sessões compilaremos o projeto [LibFreenect2](https://github.com/OpenKinect/libfreenect2) e configuraremos seu uso.

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

Após compilar o projeto e instalar os artefatos, é necessário liberar o acesso ao dispositivo para os usuários, do contrário os programas que tentarem se comunicar com o kinect precisarão ser executados em modo root.

```bash
cd libfreenect2
sudo cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```

**Após a execução dos comandos acima é necessário desconectar o kinect da porta USB e conectar novamente.**

## Testing with NiViewer2

Como primeiro teste rápido é possivel utilizar agora o NiViewer2 para obter dados do kinect.

```bash
sudo apt install openni2-utils # caso não tenha sido instalado ainda
NiViewer2
```

Após a execução dos comandos acima são esperados uma tela conforme a image e a seguinte saída no terminal:

![Tela NiViewer]({{site.baseurl}}/kinect_v2_com_opencv/niviewer2.png "Tela NiViewer")


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

Feito o teste com o NiView2, a seguir está um programa em C++ utilizando a PCL e OpenCV para obter informação do sensor e exibir a núvem de pontos em uma tela interativa. Ao final está contido o vídeo com o resultado esperado ao se compilar o programa abaixo.

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

cv::VideoCapture cap(1, cv::CAP_OPENNI2); // Configura o cv::VideoCapture para abrir o dispositivo 1 utilizando a OpenNI2

void getPointCloud(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.removeAllPointClouds(); // Remove as núvens de pontos anteriores

    cv::Mat pointCloudFrame, frame;

    cap.grab(); // Obtém-se uma leitura do sensor
    cap.retrieve(pointCloudFrame, cv::CAP_OPENNI_POINT_CLOUD_MAP); // Obtendo núvem de pontos em formato matricial
    cap.retrieve(frame, cv::CAP_OPENNI_BGR_IMAGE); // Obtendo informação de cor alinhada com a núvem de pontos

    pcl::visualization::CloudViewer::ColorCloud* cloud = new pcl::visualization::CloudViewer::ColorCloud();

    for(int i = 0; i<pointCloudFrame.rows; i++) // Para cada ponto contido na matriz
    {
        for(int j = 0; j<pointCloudFrame.cols; j++) // Para cada ponto contido na matriz
        {
            cv::Vec3b color = frame.at<cv::Vec3b>(i,j); // Obtém a informação de cor
            pcl::PointXYZRGB point(color[2], color[1], color[0]); // Cria um ponto no formato da PCL passando a informação da cor contida nesse ponto

            cv::Vec3f coords = pointCloudFrame.at<cv::Vec3f>(i,j); // Obtém as coordenadas deste ponto
            point.x = coords[0]; // Adiciona a coordenada ao ponto
            point.y = coords[1]; // Adiciona a coordenada ao ponto
            point.z = coords[2]; // Adiciona a coordenada ao ponto

            cloud->push_back(point); // Adiciona o ponto na núvem
        }
    }

    viewer.addPointCloud(pcl::visualization::CloudViewer::ColorCloud::ConstPtr(cloud)); // Envia a núvem de pontos para ser renderizada pelo visualizador
}


int main(int argc, char** argv)
{

    if(!cap.isOpened())
       return -1;

    cap.set(cv::CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1); // Configurando OpenNI para gerar núvem de pontos com informações de cores


    pcl::visualization::CloudViewer viewer("point cloud"); // Inicializa um visualizador de núvens de pontos
    viewer.runOnVisualizationThread(getPointCloud); // Define a função utilizada para renderizar a cena

    while(!viewer.wasStopped()); // Aguarda a conclusão do visualizador de núvens de pontos

    cap.release(); // Encerra a captura do dispositivo
    
    return 0;

}
```

## Result
<div style="position:relative;width:100%;padding-bottom:56.25%">
<iframe style="position:absolute;top:0;left:0;width:100%;height:100%;border:0" src="https://www.youtube.com/embed/199c-im-FOM" allowfullscreen></iframe>
</div>

<div>
<h3>
<p>In case of any doubts send me a <a href="mailto:matheusbarcelosoliveira@gmail.com">email</a>,</p>
<p><a href="https://github.com/mathbarc">Matheus Barcelos de Oliveira</a><br/>
Computer Engineer</p>
</h3>
</div>
