#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "ultimateTrackerThread.h"

IncomingEvent eventFromBottle(const Bottle &b)
{
    IncomingEvent ie;
    ie.fromBottle(b);
    return ie;
};

ultimateTrackerThread::ultimateTrackerThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    stateFlag = 0;

    motionCUTBlobs = new BufferedPort<Bottle>;
    motionCUTPos.resize(2,0.0);
}

bool ultimateTrackerThread::threadInit()
{
    motionCUTBlobs -> open(("/"+name+"/mCUT:i").c_str());
}

void ultimateTrackerThread::run()
{
    // state #01: check the motionCUT and see if there's something interesting in IT

    switch (stateFlag)
    {
        case 0:
            printMessage(0,'Looking for motion');
            stateFlag++;
            break;
        case 1:
            if (motionCUTBottle = motionCUTPort.read(false))
            {
                if (motionCUTBottle!=NULL)
                {
                    motionCUTPos.zero();
                    Bottle *blob;
                    // Let's process the blob with the maximum size
                    blob = motionCUTBottle->get(blobidx).asList();
                    motionCUTPos(0) = blob->get(0).asInt();
                    motionCUTPos(1) = blob->get(1).asInt();
                    if stabilityCheck()
                    {
                        printMessage(0,'Initializing tracker');
                        initializeTracker();
                        stateFlag++
                    }
                }
            }
            break;
        case 2:
            printMessage(0,'Initializing: looking for motion');
            stateFlag++;
            break;
        default:
            printMessage(0,"ERROR!!! doubleTouchThread should never be here!!!\nStep: %d",step);
            delay(1);
            break;
}


int ultimateTrackerThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void ultimateTrackerThread::threadRelease()
{
    // printMessage(0,"Closing ports..\n");
    //     closePort(inputPortTest);
    //     printMessage(1,"    inputPortTest successfully closed!\n");
}

// empty line to make gcc happy
