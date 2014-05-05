/* DOUBLE TOUCH v. 0.2
 * Copyright (C) 2013 RobotCub Consortium
 * Author:  Alessandro Roncone
 * email:   alessandro.roncone@iit.it
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
\defgroup visuoTactileWrapperModule visuoTactileWrapperModule

@ingroup periPersonalSpace

A module that handles the visuoTactile Receptive Fields

Date first release: 23/10/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the VisuoTactile ReceptiveFields on the iCub.

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default visuoTactileWrapper).

--robot      \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--rate   \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.

--record     \e rec
- If to record data or not. The double touch slightly differs according to this
  parameter.

--color      \e col
- Robot color (black or white or whatever) - is MANDATORY if a recording session
  with rec==2 is started

--type       \e type
- Type of the task (right now it can be either "R2L" or "L2R" or "both")

--filename   \e file
- The name of the file to be saved in case of a recording session.
  Default 'calibration.txt'. A date is appended at the beginning for completeness.

\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it reads the skinContacts from the skinManager.

- <i> /<name>/records:o </i> it prints out some data about the state of the
  recording.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze).

\author: Alessandro Roncone
*/ 

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string> 

#include "vtWThread.h"
#include "iCub/periPersonalSpace/parzenWindowEstimator.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup visuoTactileWrapperModule
*
* The module that achieves the visuoTactileWrapper task.
*  
*/
class visuoTactileWrapper: public RFModule 
{
private:
    vtWThread *vtWThrd;
    //SceneFlow* sceneFlow;

    string robot,name;
    int verbosity,rate;

    int step;

public:
    visuoTactileWrapper()
    {
        vtWThrd   =     0;
        step      =    -1;
    }

    bool configure(ResourceFinder &rf)
    {
        name  = "visuoTactileWrapper";
        robot = "icub";

        verbosity  = 0;     // verbosity
        rate       = 10;    // rate of the vtWThread

        //******************************************************
        //********************** CONFIGS ***********************

        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                cout << "Module name set to "<<name<<endl;  
            }
            else cout << "Module name set to default, i.e. " << name << endl;
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                cout << "Robot is: " << robot << endl;
            }
            else cout << "Could not find robot option in the config file; using "
                      << robot << " as default\n";

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                cout << "vtWThread verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in the" <<
                         "config file; using "<< verbosity <<" as default\n";

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                cout << "vtWThread rateThread working at " << rate << " ms\n";
            }
            else cout << "Could not find rate in the config file; using "
                      << rate << " ms as default\n";

        //******************************************************
        //*********************** THREAD **********************
            vtWThrd = new vtWThread(rate, name, robot, verbosity);
            if (!vtWThrd -> start())
            {
                delete vtWThrd;
                vtWThrd = 0;
                cout << "\nERROR!!! vtWThread wasn't instantiated!!\n";
                return false;
            }
            cout << "VISUO TACTILE WRAPPER: vtWThread istantiated...\n";

        return true;
    }

    bool close()
    {
        cout << "VISUO TACTILE RECEPTIVE FIELDS: Stopping threads.." << endl;
        if (vtWThrd)
        {
            vtWThrd->stop();
            delete vtWThrd;
            vtWThrd=0;
        }
        return true;
    }

    double getPeriod()
    {
        return 0.1;
    }

    bool updateModule()
    {
        return true;
    }
};

/**
* Main function.
*/
int main(int argc, char * argv[])
{
/*    parzenWindowEstimator pwe(0.2,10);
    for (int i = 0; i < 10; i++)
    {
        pwe.addSample(0.05);
        pwe.addSample(0.01);
    }

    for (int i = 0; i < 5; i++)
        pwe.addSample(0.03);

    pwe.addSample(0.18);
    pwe.print();
    double x=0.001;
    double fx = pwe.getF_X(x);
    printf("f(%g): %g\n", x, fx);

    fx = pwe.getF_X_scaled(x);
    printf("f(%g): %g\n", x, fx);

    return true;*/

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder moduleRF;
    moduleRF.setVerbose(true);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("visuoTactileWrapper.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context    path:  where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from       from:  the name of the .ini file (default visuoTactileWrapper.ini)." << endl;
        cout << "   --name       name:  the name of the module (default visuoTactileWrapper)." << endl;
        cout << "   --robot      robot: the name of the robot. Default icub." << endl;
        cout << "   --rate       rate:  the period used by the thread. Default 50ms." << endl;
        cout << "   --verbosity  int:   verbosity level (default 0)." << endl;
        cout << endl;
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    visuoTactileWrapper visTacWrap;
    return visTacWrap.runModule(moduleRF);
}

// empty line to make gcc happy