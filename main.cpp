#include "GxIAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include<thread>
#include<chrono>
#include<memory>
#include <mutex>
#include <queue>

#include "include/dahengFunc.h"
#include "include/CommonProcessingUnit.h"
#include "include/common.h"
#include "include/refineDisparity.h"
#include "include/guidedfilter.h"

using namespace pcl;
using namespace std;
using namespace cv;
using namespace daheng;
using namespace CommonProcessingUnit;
using namespace StereoMatching;
using namespace RefineDisparity;

//pcl::visualization::PCLVisualizer viewer1("viewer_1");

queue<ImagePair> pairs;
mutex mu;  //线程互斥对象  

bool working = true;

float cropFactor = 0.5;

cv::Mat lmapx, lmapy, rmapx, rmapy;

void cameraThread(DahengDevice& dahengDevice)
{
    while (working)
    {
        dahengDevice.acquisitionBinocularImages();
        
        Mat stereoImg(cv::Size(dahengDevice.stereoImgData.nWidth, dahengDevice.stereoImgData.nHeight), CV_8UC1, (void*)dahengDevice.stereoImgData.pImgBuf, cv::Mat::AUTO_STEP);
        int widthS = dahengDevice.stereoImgData.nWidth;
        int heightS = dahengDevice.stereoImgData.nHeight;
        Rect rectL(0, 0, widthS * cropFactor, heightS);
        Rect rectR(widthS * cropFactor, 0, widthS * cropFactor, heightS);
        
        Mat Il(stereoImg, rectR);
        Mat Ir(stereoImg, rectL);


        cv::Mat leftR, rightR;
        rectifyStereo(Il, Ir, leftR, rightR, lmapx, lmapy, rmapx, rmapy);

        ImagePair imgs;
        imgs.imgL = leftR.clone();
        imgs.imgR = rightR.clone();
        imgs.imgID = dahengDevice.stereoImgData.nFrameID;
        mu.lock();
        if (pairs.size() > 5)
        {
            pairs.pop();
        }
        pairs.push(imgs);
        mu.unlock();

        //cout <<"cameraThread: "<< "leftImgData: "<<dahengDevice.leftImgData.nFrameID << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

}




int main()
{

  

   
   

  

    //-----------------------SGM Initialization-------------------------------
   
    int disp_size = 128;

  
    
    Mat II1 = cv::imread("../image/left.png",0);
    Mat II2=  cv::imread("../image/right.png",0);
    //Mat I1,I2;
    Rect rect(0, 0,1280, 720);


    Mat I1(II1, rect);
    Mat I2(II2, rect);
    //II1.convertTo(I1, CV_8U);
    //II2.convertTo(I2, CV_8U);
   /* imshow("test", I1);
    std::cout << "sss" << I1.type() << std::endl;
    waitKey(0);*/
  
    ASSERT_MSG(!I1.empty() && !I2.empty(), "imread failed.");
    ASSERT_MSG(I1.size() == I2.size() && I1.type() == I2.type(), "input images must be same size and type.");
    ASSERT_MSG(I1.type() == CV_8U || I1.type() == CV_16U, "input image format must be CV_8U or CV_16U.");
    ASSERT_MSG(disp_size == 64 || disp_size == 128, "disparity size must be 64 or 128.");

    int width = I1.cols;
    int height = I1.rows;
    cout << I1.type() << endl;

    const int input_depth = I1.type() == CV_8U ? 8 : 16;
    const int input_bytes = input_depth * width * height / 8;
    const int output_depth = 8;
    const int output_bytes = output_depth * width * height / 8;

    sgm::StereoSGM sgm(width, height, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_CUDA2CUDA);
    device_buffer d_I1(input_bytes), d_I2(input_bytes), d_disparity(output_bytes);
    cv::Mat disparity(height, width, output_depth == 8 ? CV_8U : CV_16U);

    
   
    while (1)
    {
      
        
           
                Mat stereoImg;
                hconcat(I1, I2, stereoImg);
                cv::namedWindow("STEREO", CV_WINDOW_NORMAL);
                imshow("STEREO", stereoImg);

                //imwrite("../image/left.png", imgs.imgL);
                //imwrite("../image/right.png", imgs.imgR);
               // cv::waitKey(1);
                //rectifyStereo(imgs.imgL, imgs.imgR, leftR, rightR, lmapx, lmapy, rmapx, rmapy);
              
                cv::Mat leftR = I1.clone();
                cv::Mat rightR = I2.clone();
                cudaMemcpy(d_I1.data, leftR.data, input_bytes, cudaMemcpyHostToDevice);
                cudaMemcpy(d_I2.data, rightR.data, input_bytes, cudaMemcpyHostToDevice);
                sgm.execute(d_I1.data, d_I2.data, d_disparity.data);
                cudaMemcpy(disparity.data, d_disparity.data, output_bytes, cudaMemcpyDeviceToHost);
                if (disparity.empty())
                {
                    cout << "data empty" << endl;
                    break;
                }
                Mat result;
               

               
                drawColorDisparity(disparity,  128);
                                    
               

            int flag = cv::waitKey(1);
            if (flag == 27)
            {
                working = false;
                break;
            }

           /* if (flag == 32)
            {
                saveCloudPoint(depth, leftR);
            }
*/
     }

    

   

    return 0;
}

