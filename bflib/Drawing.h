#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
<<<<<<< HEAD
#include <string>	
#include <opencv2/opencv.hpp>
=======
#include <string>
#include <vector>
#include <cmath>
#include "BFLIB/PF.hpp"
>>>>>>> 8e4e73c3088402e16f1b714570b82d6862a38136

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
        void teste1();

        ~Drawing();
        /** Destructor for Drawing class
         */
<<<<<<< HEAD
         void function_test();

         void drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color);
        /** Method that creates particle simulations
         */ 
        
        void drawLandmarks();
        /**
         * Drawing Landmarks in map
        */
=======
        void teste2();
        void drawParticles(cv::Mat& image, const vector< Robot::State >& PS, const cv::Scalar& color)
        /** Method that creates particle simulations
         */
        void teste33333333();

>>>>>>> 8e4e73c3088402e16f1b714570b82d6862a38136

};


#endif
