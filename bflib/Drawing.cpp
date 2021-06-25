#include "Drawing.h"

const float PI2 = 3.14159265358979f;
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
    float aperture = PI2/6;
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

void Drawing::drawLandmarks(cv::Mat &image, vector<Vector3d> &PS, const cv::Scalar &color)
{
    for (int i = 0; i < PS.size(); i++)
    {
        cv::circle(image, cv::Point(PS[i][0], PS[i][1]), 4, color, CV_FILLED);
    }
}
void Drawing::drawPath(cv::Mat &image, const Robot::State &XR, const vector<double> &X, const vector<double> &Y, const cv::Scalar &color, bool strip)
{
    int S = min(X.size(), Y.size());
    vector<cv::Point> points(S);
    for (int i = 0; i < S; i++)
    {
        points[i] = cv::Point(10 + 100 * X[i], 10 + 100 * Y[i]);
    }
    if (strip)
    {
        for (int i = 0; i < S - 1; i += 4)
        {
            cv::line(image, points[i], points[i + 1], color, 1);
        }
    }
    else
        cv::polylines(image, points, false, color, 1);
    cv::circle(image, points.back(), 5, color, CV_FILLED);

    cv::Point pf;
    pf.x = (10 + 100 * XR[0]) + 10 * cos(XR[2]);
    pf.y = (10 + 100 * XR[1]) + 10 * sin(XR[2]);
    cv::line(image, points.back(), pf, color, 2);
}

void Drawing::drawFieldCenter(cv::Mat &image)
{   
    cv::circle(image, cv::Point(244,170), 45, cv::Scalar(0,0,0), 2);
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
