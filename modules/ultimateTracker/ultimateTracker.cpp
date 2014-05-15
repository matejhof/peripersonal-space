/* VISUO TACTILE RECEPTIVE FIELDS v. 1.0
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
\defgroup ultimateTrackerModule ultimateTrackerModule

@ingroup periPersonalSpace

THE ultimate tracker. It tracks everything without any hiccup!

Date first release: 23/05/2014

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is nothing but a tracker. But it's the ultimate one!

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default ultimateTracker).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--rate          \e rate
- The period used by the thread. Default 100ms.

\section portsc_sec Ports Created

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze, Debian Wheezy).

\author: Alessandro Roncone and Ugo Pattacini
*/ 

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string> 

#include "ultimateTrackerThread.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup ultimateTrackerModule
*
* The module that achieves the ultimateTracker task.
*  
*/
class ultimateTracker: public RFModule 
{
private:
    ultimateTrackerThread *ultTrckrThrd;

    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot,name;
    int verbosity,rate;

public:
    ultimateTracker()
    {
        ultTrckrThrd    = 0;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB4('s','a','v','e'):
                {
                    int res=Vocab::encode("saved");
                    if (ultTrckrThrd -> save())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('l','o','a','d'):
                {
                    int res=Vocab::encode("loaded");
                    if (ultTrckrThrd -> load())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('r','e','s','e'):
                {
                    ultTrckrThrd -> resetParzenWindows();
                    reply.addVocab(ack);
                    return true;
                }
                case VOCAB4('s','t','o','p'):
                {
                    ultTrckrThrd -> stopLearning();
                    reply.addVocab(ack);
                    return true;
                }
               case VOCAB4('r','e','s','t'):
                {
                    ultTrckrThrd -> restoreLearning();
                    reply.addVocab(ack);
                    return true;
                }     
                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        name  = "ultimateTracker";
        robot = "icub";

        verbosity  = 0;      // verbosity
        rate       = 100;    // rate of the ultimateTrackerThread

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
                cout << "ultimateTrackerThread verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in " <<
                         "config file; using "<< verbosity <<" as default\n";

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                cout << "ultimateTrackerThread rateThread working at " << rate << " ms\n";
            }
            else cout << "Could not find rate in the config file; using "
                      << rate << " ms as default\n";

        //******************************************************
        //*********************** THREAD **********************
            ultTrckrThrd = new ultimateTrackerThread(rate, name, robot, verbosity);
            if (!ultTrckrThrd -> start())
            {
                delete ultTrckrThrd;
                ultTrckrThrd = 0;
                cout << "\nERROR!!! ultimateTrackerThread wasn't instantiated!!\n";
                return false;
            }
            cout << "ULTIMATE TRACKER: ultimateTrackerThread istantiated...\n";

        //******************************************************
        //************************ PORTS ***********************
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);

        return true;
    }

    bool close()
    {
        cout << "ULTIMATE TRACKER: Stopping thread.." << endl;
        if (ultTrckrThrd)
        {
            ultTrckrThrd->stop();
            delete ultTrckrThrd;
            ultTrckrThrd=0;
        }
        return true;
    }

    double getPeriod()
    {
        return 0.05;
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
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder moduleRF;
    moduleRF.setVerbose(false);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("ultimateTracker.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context    path:   where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from       from:   the name of the .ini file (default ultimateTracker.ini)." << endl;
        cout << "   --name       name:   the name of the module (default ultimateTracker)." << endl;
        cout << "   --robot      robot:  the name of the robot. Default icub." << endl;
        cout << "   --rate       rate:   the period used by the thread. Default 50ms." << endl;
        cout << "   --verbosity  int:    verbosity level (default 0)." << endl;
        cout << "   --taxelsFile string: the file from which load and save taxels (default taxels2D.ini)." << endl;
        cout << endl;
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    ultimateTracker ultTrack;
    return ultTrack.runModule(moduleRF);
}

// empty line to make gcc happy
