#include "Drawing.h"

Drawing::Drawing()
{
    
}
Drawing::~Drawing()
{

}

void Drawing::drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color)
{
    for(int i = 0; i < PS.size(); i++)
    {
        cv::circle(image, cv::Point(10 + 100 * PS[i][0], 10 + 100 * PS[i][1]), 2, color, CV_FILLED);
    }
}

void Drawing::drawSensor(cv::Mat &image, const Robot::State &X, const cv::Scalar &color)
{
    cv::Point2f xSnow, xR,xMid;
    float aperture = PI/6;
    float rad = 100;
    xR.x = (10 + 100 * X[0]);
    xR.y = (10 + 100 * X[1]);

    xMid.x = xR.x + rad * cos(X[2]);
    xMid.y = xR.y + rad * sin(X[2]);
    //cv::circle(image,xR,2, color, 0);
    for (int i = 0 ; i < int(aperture*57.29f); i++)
    {
        xSnow.x = xR.x + rad * cos(X[2]  + 0.01745f * i - aperture*0.5);
        xSnow.y = xR.y + rad * sin(X[2]  + 0.01745f * i - aperture*0.5);

        cv::line(image, xR, xSnow, cv::Scalar(0, 255, 255, 0), 4, CV_AA, 0);
        //return cv::line(image,xR,xSnow,cv::Scalar(0,255,255,0), 4,CV_AA,0);
    }
    cv::line(image, xR, xMid, cv::Scalar(0, 0, 0, 0), 1, CV_AA, 0);
}