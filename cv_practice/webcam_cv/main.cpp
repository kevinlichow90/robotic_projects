//
//  main.cpp
//  webcam_cv
//
//  Created by Kevin Chow on 3/17/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

/// Global Variables
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

/// Matrices to store images
cv::Mat src1;
cv::Mat dst;

void on_trackbar(int, void*) {
    alpha = (double) alpha_slider/alpha_slider_max;
    dst = src1*alpha;
    //std::cout << dst << "\n";
    cv::imshow("Alpha Scaled Image", dst);
    
}

int main()
{
    // initialize webcam video capture
    cv::VideoCapture cap;
    cap.open(0);
    
    if( !cap.isOpened() )
    {
        std::cerr << "***Could not initialize capturing...***\n";
        std::cerr << "Current parameter's value: \n";
        return -1;
    }
    
    // create video window and trackbar
    cv::Mat frame;
    cv::namedWindow("Alpha Scaled Image");
    char TrackbarName[50];
    sprintf(TrackbarName, "Alpha x %d", alpha_slider_max);
    
    
    while(1){
        cap >> src1;
        if(src1.empty()){
            std::cerr<<"frame is empty"<<std::endl;
            break;
        }
        
        // currently the video pauses when the trackbar slider is clicked
        cv::createTrackbar(TrackbarName, "Alpha Scaled Image", &alpha_slider, alpha_slider_max, on_trackbar);
        on_trackbar( alpha_slider, 0 );
        cv::waitKey(10);
    }
    
    
    return 1;
}
