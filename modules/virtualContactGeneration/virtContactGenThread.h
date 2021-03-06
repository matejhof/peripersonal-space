/*
 * Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Matej Hoffmann
 * email:  matej.hoffmann@iit.it
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

#ifndef __VIRTCONTACTGENTHREAD_H__
#define __VIRTCONTACTGENTHREAD_H__

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdarg.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>

#include <iCub/skinDynLib/skinContact.h>
 #include <iCub/skinDynLib/skinPart.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iCub/periPersonalSpace/utils.h>

#define VIRT_CONTACT_PRESSURE 100

using namespace std;

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::skinDynLib;

class virtContactGenerationThread: public RateThread
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
    // Resource finder used to find for files and configurations:
    ResourceFinder* rf;
    //the period used by the thread. 
    int threadPeriod; 
    // type of selection of contacts - e.g. random
    string type;
    
    //based on .ini file, contains a list of skin parts that will be part of the virtual contact generation
    vector<SkinPart> activeSkinPartsNames;
    map<SkinPart,string> skinPartPosFilePaths;
       
    /***************************************************************************/
    // INTERNAL VARIABLES
    
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
    
    // Port with the fake contacts:
    BufferedPort<skinContactList> *skinEventsOutPort;
    
    //will contain actual skin parts with list of taxels and their positions
    map<SkinPart,iCub::skinDynLib::skinPart> activeSkinParts;
    int skinPartIndexInVector;
    SkinPart skinPartPickedName;
    iCub::skinDynLib::skinPart skinPartPicked;
    int taxelPickedIndex;
    Taxel taxelPicked;
    std::vector<unsigned int> taxelIDinList;
   
    /**
    * Initializes vector of skin parts - activeSkinParts based on the activeSkinPartsNames. 
    * Uses the class members (activeSkinPartsNames and activeSkinParts) and modifies activeSkinParts - fills it with appropriate taxel objects etc.
    */
    int initSkinParts();
       
     /**
    * Prints the initialized skin parts into files - for debugging purposes. Only if verbosity > 5.
    */
    void printInitializedSkinParts();
    
    /**
    * Prints a message according to the verbosity level:
    * @param l will be checked against the global var verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    
public:
    // CONSTRUCTOR
    virtContactGenerationThread(int _rate, const string &_name, const string &_robot,
                                int _v, const string &_type, const vector<SkinPart> &_activeSkinPartsNames,
                                const map<SkinPart,string> &_skinPartPosFilePaths);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

};

#endif
