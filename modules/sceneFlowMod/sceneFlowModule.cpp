/* 
 * Copyright (C) 2011 RobotCub Consortium
 * Author: Sean Ryan Fanello, Ilaria Gori
 * email:   sean.fanello@iit.it, ilaria.gori@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
\defgroup sceneFlowModule sceneFlowModule

@ingroup icub_stereoVision

A test module for the image of the world.

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello, Ilaria Gori
 
Date: 19/12/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is an example module for the SceneFlow computation.

\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--configDisparity \e file.ini 
- The parameter \e file.ini specifies the cameras config file (e.g icubEyes.ini).

--denseFlow \e val 
- The parameter \e val specifies the flow type: dense if val=1; sparse if val=0.

--leftCamera \e left
- The parameter \e left specifies the left camera port (e.g. /icub/camcalib/left/out).

--rightCamera \e right
- The parameter \e right specifies the right camera port (e.g. /icub/camcalib/right/out).

\section portsc_sec Ports Created


\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7.

\author Sean Ryan Fanello, Ilaria Gori
*/ 

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cv.h>
#include <highgui.h>
#include <iCub/ctrl/math.h>
#include "sceneFlowThreadMod.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;

class sceneFlowModule: public yarp::os::RFModule
{

private:
    // SceneFlow* sceneFlow;
    sceneFlowThreadMod* sceneFlow;

public:

    bool configure(ResourceFinder &_rf)
    {
        sceneFlow=new sceneFlowThreadMod(_rf);
        if(sceneFlow->isOpen())
        {
            sceneFlow->start();
            Time::delay(0.5);
            return true;
        }
        else
        {
            delete sceneFlow;
            return false;
        }
        namedWindow("flowField");
    };

    bool close()
    {
        delete sceneFlow;   
        return true;
    };

    bool updateModule()
    {
        if(isStopping())
            return false;

        // Whole Flow Access
        int u=160;
        int v=120;
        Mat flow3D;
        sceneFlow->getSceneFlow(flow3D);
        Point3f Pflow3D;

        if(!flow3D.empty())
        {
            Pflow3D.x=flow3D.ptr<float>(v)[2*u]; // Flow DeltaX
            Pflow3D.y=flow3D.ptr<float>(v)[2*u+1]; // Flow DeltaY
            Pflow3D.z=flow3D.ptr<float>(v)[2*u+2]; // Flow DeltaZ
            
            fprintf(stdout,"3D Motion of Pixel (%i,%i): (%f, %f, %f) \n",u,v,Pflow3D.x,Pflow3D.y,Pflow3D.z);

            // Compute and show motion field
            IplImage* flowField=sceneFlow->draw2DMotionField();
            
            if(flowField!=NULL)
                cvShowImage("flowField", flowField);
            cvWaitKey(5);

            cvReleaseImage(&flowField);
        }


        return true;
    };

    double getPeriod()
    {
        return 0.1;
    };

    bool interruptModule()
    {
        fprintf(stdout, "Closing Scene Flow...\n");
        sceneFlow->suspend();
        sceneFlow->close();     
        return true;
    };
};

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("sceneFlowModule/conf");
    rf.configure("ICUB_ROOT",argc,argv);
    sceneFlowModule mod;

    return mod.runModule(rf);
}