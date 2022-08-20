#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>     
#include <iostream>        
#include "marker.h"
#include <opencv2/imgproc/types_c.h>
#include <zbar.h>

using namespace std;        
using namespace zbar;  //添加zbar名称空间      
using namespace cv;     

int main(int argc,char*argv[])      
{   
    
    cout<< "CV_VERSION: " << CV_VERSION << endl;
    //相机标定的参数-------
    double fx = 412.433229;
	double cx = 318.310004;
	double fy = 414.182775;
	double cy = 236.769192;
	double k1 = -0.320394;
	double k2 =  0.108028;
	double p1 = -0.000993;
	double p2 =  0.001297;
	double k3 =  0;
    Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
    //相机标定的参数-------
    //二维码的边长
	double marker_size = 5.0; //单位  cm
    //zbar::ImageScanner
    ImageScanner scanner;        
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);      
    //仅仅创建一个捕获对象，而不提供任何关于打开的信息。
    cv::VideoCapture inputVideo;
    //如果只有1个摄像机，那么就是0，如果系统中有多个摄像机，那么只要将其向上增加即可。6为realsense
    inputVideo.open(6);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,720);
    
    while (inputVideo.grab()) 
    {
        cv::Mat image;
       
        
		inputVideo.retrieve(image);//抓取视频中的一张照片
		flip(image,image,1);//  水平方向镜像
        Mat imageGray;        
        cvtColor(image,imageGray,CV_RGB2GRAY);   
        int width = imageGray.cols;        
        int height = imageGray.rows;  

        uchar *raw = (uchar *)imageGray.data;           
        Image imageZbar(width, height, "Y800", raw, width * height);  


        scanner.scan(imageZbar); //扫描QR码     

        Image::SymbolIterator symbol = imageZbar.symbol_begin();    
        if(imageZbar.symbol_begin()==imageZbar.symbol_end())    
        {    
            cout<<"can't detect QR code！"<<endl;    
        }    
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        std::vector<cv::Vec3d> rvecs, tvecs;
        for(int i = 0;symbol != imageZbar.symbol_end();++symbol)      
        {        
            cout<<"type："<<endl<<symbol->get_type_name()<<endl<<endl;      
            cout<<"data："<<endl<<symbol->get_data()<<endl<<endl; 
            cout<<"data_length："<<endl<<symbol->get_data_length()<<endl<<endl; 
            cout<<"location_size："<<endl<<symbol->get_location_size()<<endl<<endl; 
             
            std::vector<cv::Point2f> corner;
            corner.push_back(cv::Point2f(symbol->get_location_x(0),symbol->get_location_y(0)));
            corner.push_back(cv::Point2f(symbol->get_location_x(3),symbol->get_location_y(3)));
            corner.push_back(cv::Point2f(symbol->get_location_x(2),symbol->get_location_y(2)));
            corner.push_back(cv::Point2f(symbol->get_location_x(1),symbol->get_location_y(1)));
            corners.push_back(corner);
            ids.push_back(i);
            i++; 
        }        

        //求解旋转向量rvecs和平移向量tvecs
        myEstimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (int i = 0; i < ids.size(); i ++)
        {
            // cv::aruco::drawDetectedMarkers(image, corners, ids);//绘制检测到的靶标的框
            myDrawDetectedMarkers(image, corners, ids,Scalar(100, 0, 255));//绘制检测到的靶标的框
            // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5);
            myDrawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5, 3);
            // drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5, 3);//opencv 3.3.1 没有 4.2.1有
            cout<<"  T :"<<tvecs[i]<<endl;
            cout<<"  R :"<<rvecs[i]<<endl;
        }
        imshow("Source Image",image);    
        waitKey(1);  
     
    }

    return 0;    
}     

