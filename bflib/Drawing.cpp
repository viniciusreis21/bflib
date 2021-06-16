#include "Drawing.h"
#include <bflib/PF.hpp>
#include <iostream>
#include <vector>
#include <cmath>

Drawing::Drawing()
{   
    lines = {Vector4d(0.000, 0.000, 4.680, 0.000),
            Vector4d(0.000, 0.000, 0.000, 3.200),
            Vector4d(0.000, 3.200, 4.680, 3.200),
            Vector4d(4.680, 0.000, 4.680, 3.200),
            Vector4d(2.340, 0.000, 2.340, 3.200), // linha do meio de campo
            Vector4d(0.000, 1.010, 0.312, 1.010),
            Vector4d(0.000, 2.190, 0.312, 2.190),
            Vector4d(0.312, 1.010, 0.312, 2.190),
            Vector4d(4.680, 1.010, 4.368, 1.010),
            Vector4d(4.680, 2.190, 4.368, 2.190),
            Vector4d(4.368, 1.010, 4.368, 2.190),
            Vector4d(0.650, 1.600, 0.702, 1.600), // linha horizontal da cruz esquerda
            Vector4d(0.676, 1.574, 0.676, 1.626),
            Vector4d(3.978, 1.600, 4.030, 1.600),
            Vector4d(4.004, 1.574, 4.004, 1.626)};
    // Vector4d(0.000,  2.280,   0.920,   2.280),
    // Vector4d(0.920,  2.280,   0.920,   3.200),
    // Vector4d(4.190,  2.850,   4.680,   2.850),
    // Vector4d(4.190,  2.850,   4.190,   3.200),
    // Vector4d(4.030,  0.000,   4.680,   0.650)

}

Drawing::~Drawing()
{

}
void Drawing::function_test()
{
    
}

void Drawing::drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color)
{
    for(int i = 0; i < PS.size(); i++)
    {
        cv::circle(image, cv::Point(10 + 100 * PS[i][0], 10 + 100 * PS[i][1]), 2, color, CV_FILLED);
    }
}
void Drawing::drawLandmarks()
{

}


//-----------Pedro----------
void Drawing::drawLines(cv::Mat &image, const cv::Scalar &color)
{
    for (int i = 0; i < this->lines.size(); i++)
    {
        cv::line(image,
                 cv::Point(10 + this->lines[i](0) * 100, 10 + this->lines[i](1) * 100),
                 cv::Point(10 + this->lines[i](2) * 100, 10 + this->lines[i](3) * 100),
                 color, 2);
    }
}
void Drawing::teste3()
{

}