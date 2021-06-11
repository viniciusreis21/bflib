#include "Drawing.h"

Drawing::Drawing()
{
    
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
