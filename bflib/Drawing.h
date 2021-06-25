#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <string>	
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "PF.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

/** This is a Drawing class for simulation and tests purpose.
* The main objective of this class is to generate all the drawing functions for the purposes of simulating our model.
*
*/
typedef PF<double, 3, 2, 2, 9> Robot;

class Drawing
{
    public:
        vector<Vector4d> lines;

        Drawing();
        /** Constructor for Drawing class
         */

        ~Drawing();
        /** Destructor for Drawing class
         */
        
        void drawLandmarks(cv::Mat &image, vector<Vector3d> &PS, const cv::Scalar &color);
        /**
         * Drawing Landmarks in map
        */
        void drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color);

        /** Method that creates particle simulations
         */
        void drawPath(cv::Mat &image, const Robot::State &XR, const vector<double> &X, const vector<double> &Y, const cv::Scalar &color, bool strip);
        void drawFieldCenter(cv::Mat &image);
        

        // ---------------Pedro---------------

        void drawLines(cv::Mat &image, const cv::Scalar &color);

        void drawSensor(cv::Mat &image, const Robot::State &X, const cv::Scalar &color);

};


#endif
