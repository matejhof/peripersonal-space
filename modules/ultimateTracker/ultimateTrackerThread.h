/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
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
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#ifndef __VTRFTHREAD_H__
#define __VTRFTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <set>
#include <list>

#include <cv.h>
#include <highgui.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/periPersonalSpace/utils.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class ultimateTrackerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int    stateFlag;
    double timeNow;

    BufferedPort<Bottle>       *motionCUTBlobs;  // port for reading from motionCUT
    Bottle                     *motionCUTBottle; // bottle used for the port
    Vector                      motionCUTPos;    // current position of the center of the blob
    vector <yarp::sig::Vector>  oldMcutPoss;     // old positions

    BufferedPort<Bottle>  *templatePFTrackerTarget;  // port for reading from motionCUT
    Bottle                *templatePFTrackerBottle;  // bottle used for the port
    Vector                 templatePFTrackerPos;     // current position of the center of the blob

    bool noInput();

    bool processMotion();


    /**
    * Checks the stability of the motionCUT's blob center. It returns true if 
    * there are at least 10 past events and the new sample differs from the past average
    * for less than 1 pixel on the u axis and 1 on the v axis
    **/
    bool stabilityCheck();

    bool initializeTracker();

    bool readFromTracker();

    bool getPointFromStereo();




    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    ultimateTrackerThread(int _rate, const string &_name, const string &_robot, int _v);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
};

#endif

// empty line to make gcc happy
