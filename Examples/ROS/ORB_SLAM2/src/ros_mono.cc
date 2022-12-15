/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include"../../../include/System.h" // Gang ： 这里是ORB-SLAM的头文件，不要用这种方式，而要放在CMmake里

using namespace std;
using namespace cv;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    //Gang 图像话题
    //ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
//https://www.likecs.com/show-277273.html
void find_bar_area(const Mat &image){
    Mat imageGray,imageGaussian;
    Mat imgDraw = image.clone(); //不要影响原来的图片
    Mat imageSobelX,imageSobelY,imageSobleOut;
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,8);

    // 转换为灰度图
    cvtColor(image,imageGray,CV_RGB2GRAY);

    // 高斯平滑滤波
    GaussianBlur(imageGray,imageGaussian,Size(3,3),0);
    imshow("3 gaussian",imageGaussian);

    //求得水平方向和垂直方向的梯度差
    Mat imageX16S, imageY16S;
    Sobel(imageGaussian,imageX16S,CV_16S,1,0,3,1,0,4);
    Sobel(imageGaussian,imageY16S,CV_16S,0,1,3,1,0,4);

    convertScaleAbs(imageX16S,imageSobelX,1,0);
    convertScaleAbs(imageY16S,imageSobelY,1,0);
    imageSobleOut = imageSobelX + imageSobelY;
    imshow("4 x 方向梯度",imageSobelX);
    imshow("4 y 方向梯度",imageSobelY);
    imshow("4 xy 方向梯度和",imageSobleOut);

    // 均值滤波，消除噪声
    blur(imageSobleOut,imageSobleOut,Size(3,3));
    imshow("5  均值滤波",imageSobleOut);

    // 2值化
    Mat imageSobleOutThreshold;
    threshold(imageSobleOut,imageSobleOutThreshold,180,255,CV_THRESH_BINARY);
    imshow("6 二值化",imageSobleOutThreshold);

    // 7 闭运算，填充条码间隙
    Mat element = getStructuringElement(0,Size(7,7));
    morphologyEx(imageSobleOutThreshold,imageSobleOutThreshold,MORPH_CLOSE,element);
    imshow("7 闭运算",imageSobleOutThreshold);

    // 8 腐蚀，去除孤立点
    erode(imageSobleOutThreshold,imageSobleOutThreshold,element);
    imshow("8 腐蚀",imageSobleOutThreshold);

    //9. 膨胀，填充条形码间空隙，根据核的大小，有可能需要2~3次膨胀操作 
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);  
    imshow("9.膨胀",imageSobleOutThreshold);
    vector<vector<Point>> contours;
    vector<Vec4i> hiera;

    //10.通过findContours找到条形码区域的矩形边界
    // 可能有多个矩形
    findContours(imageSobleOutThreshold,contours,hiera,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    long qrcode_area_min = 6000;
    long qrcode_area_max = 10000;
    for(int i=0;i<contours.size();i++){
        IplImage imageText = imgDraw;
        Rect rect = boundingRect((Mat)contours[i]);
        if (rect.area() > qrcode_area_min && rect.area() < qrcode_area_max ){
            rectangle(imgDraw,rect,Scalar(255),2);
            std::string msg = ""+  std::to_string(rect.area());
            cvPutText(&imageText,msg.c_str(),rect.tl(),&font,cvScalar(0,255,0,1));
        }
    }
    imshow("10.找出二维码矩形区域",imgDraw);

}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    find_bar_area(cv_ptr->image);
    mpSLAM->TrackMonocular(dst,cv_ptr->header.stamp.toSec());
}


