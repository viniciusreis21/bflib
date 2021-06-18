#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <string>	
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "BFLIB/PF.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

/** This is a Drawing class for simulation and tests purpose.
* The main objective of this class is to generate all the drawing functions for the purposes of simulating our model.
*
*/

class Drawing
{
    public:
        Drawing();
        /** Constructor for Drawing class
         */

        ~Drawing();
        /** Destructor for Drawing class
         */
        
        void drawLandmarks();
        /**
         * Drawing Landmarks in map
        */
        void drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color)

        /** Method that creates particle simulations
         */

        // ---------------Pedro---------------

        vector<Vector4d> lines;
    
        //void setLines(const vector<Vector4d>);
        void drawLines(cv::Mat &image, const cv::Scalar &color);

        void drawSensor(cv::Mat &image, const Robot::State &X, const cv::Scalar &color);

};


#endif
