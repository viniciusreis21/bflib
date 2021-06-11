#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

/** This is a Drawing class for simulation and tests purpose.
* The main objective of this class is to generate all the drawing functions for the purposes of simulating our model.
* 
*/ 

class Drawing
{
    public:
        vector<Vector4d> lines;

        Drawing(); //Constructor for Drawing class
        ~Drawing(); //Destructor for Drawing class
    
        //void setLines(const vector<Vector4d>);
        void drawLines(cv::Mat &image, const cv::Scalar &color);

};


#endif