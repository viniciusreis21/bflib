#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <string>	
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
         void function_test();

         void drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color);
        /** Method that creates particle simulations
         */ 
        
        void drawLandmarks();
        /**
         * Drawing Landmarks in map
        */

};


#endif