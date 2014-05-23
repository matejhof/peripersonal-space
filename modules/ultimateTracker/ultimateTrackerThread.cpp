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
    timeNow = yarp::os::Time::now();

    motionCUTBlobs = new BufferedPort<Bottle>;
    motionCUTPos.resize(2,0.0);
    
    templatePFTrackerTarget = new BufferedPort<Bottle>;
    templatePFTrackerPos.resize(2,0.0);

    SFMPos.resize(3,0.0);

    int order  = 4;
    kalmanA      = eye(order);
    kalmanA(0,1) = 0.01;
    kalmanA(0,2) = 0.0001;
    kalmanA(1,2) = 0.01;
    kalmanA(1,3) = 0.0001;
    kalmanA(2,3) = 0.01;

    kalmanH      = zeros(1,order);
    kalmanH(0,0) = 1;

    kalmanQ = 0.00001 * eye(order);
    kalmanP = 0.00001 * eye(order);

    // Threshold is set to chi2inv(0.95,1)
    kalmanThres = 3.8415;

    kalmanR      = eye(1);
    kalmanR(0,0) = 0.01;

    for (int i = 0; i < 3; i++)
    {
        posVelKalman.push_back(Kalman(kalmanA,kalmanH,kalmanQ,kalmanR));
    }
}

bool ultimateTrackerThread::threadInit()
{
    motionCUTBlobs -> open(("/"+name+"/mCUT:i").c_str());
    templatePFTrackerTarget -> open(("/"+name+"/pfTracker:i").c_str());
    SFMrpcPort.open(("/"+name+"/SFM:o").c_str());

    Network::connect("/motionCUT/blobs:o",("/"+name+"/mCUT:i").c_str());
    Network::connect("/templatePFTracker/target:o",("/"+name+"/pfTracker:i").c_str());
    Network::connect(("/"+name+"/SFM:o").c_str(),"/SFM/rpc");

    return true;
}

void ultimateTrackerThread::run()
{
    switch (stateFlag)
    {
        case 0:
            printMessage(0,"Looking for motion\n");
            timeNow = yarp::os::Time::now();
            oldMcutPoss.clear();
            stateFlag++;
            break;
        case 1:
            // state #01: check the motionCUT and see if there's something interesting
            if (processMotion())
            {
                printMessage(0,"Initializing tracker\n");
                timeNow = yarp::os::Time::now();
                initializeTracker();
                stateFlag++;
            }
            break;
        case 2:
            printMessage(0,"Initializing Kalman filter...\n");
            readFromTracker();
            if (getPointFromStereo())
            {
                manageKalman(0);
                stateFlag++;
            }
            break;        
        case 2:
            printMessage(2,"Reading from tracker and SFM...\n");
            readFromTracker();
            if (getPointFromStereo())
            {
                manageKalman(1);
            }
            else
                manageKalman(-1);
            break;
        default:
            printMessage(0,"ERROR!!! ultimateTrackerThread should never be here!!!\nState: %d\n",stateFlag);
            Time::delay(1);
            break;
    }
}

bool ultimateTrackerThread::manageKalman(const bool init)
{
    // If init is equal to 0, the filter has to be initialized,
    // otherwise we need the predict/update stuff
    if (init == 0)
    {
        for (int i = 0; i < 3; i++)
        {
            posVelKalman(i).init(SFMPos(i),kalmanP);
        }
        return true;
    }
    else if (init == 1)
    {
        return true;
    }

    return false;
}

bool ultimateTrackerThread::noInput()
{
    if (yarp::os::Time::now() - timeNow > 1.0)
    {
        timeNow = yarp::os::Time::now();
        return true;
    }
    return false;
}

bool ultimateTrackerThread::processMotion()
{
    if (motionCUTBottle = motionCUTBlobs->read(false))
    {
        if (motionCUTBottle!=NULL)
        {
            timeNow = yarp::os::Time::now();
            motionCUTPos.zero();
            Bottle *blob;
            // Let's process the blob with the maximum size
            blob = motionCUTBottle->get(0).asList();
            motionCUTPos(0) = blob->get(0).asInt();
            motionCUTPos(1) = blob->get(1).asInt();
            // If the blob is stable, return true.
            if (stabilityCheck())
                return true;
        }
    }
    
    if (noInput())
    {
        timeNow = yarp::os::Time::now();
        oldMcutPoss.clear();
    }
    return false;
}

bool ultimateTrackerThread::stabilityCheck()
{
    oldMcutPoss.push_back(motionCUTPos);

    if (oldMcutPoss.size()>20)
    {
        oldMcutPoss.erase(oldMcutPoss.begin());  //remove first element
        Vector avgPos(2,0.0);

        for (int i = 0; i < oldMcutPoss.size(); i++)
        {
            avgPos(0) += oldMcutPoss[i](0);
            avgPos(1) += oldMcutPoss[i](1);
        }
        avgPos = avgPos/oldMcutPoss.size();

        if (motionCUTPos(0)-avgPos(0)<1.0 && motionCUTPos(1)-avgPos(1)<1.0)
        {
            return true;
        }
        else
            oldMcutPoss.clear();
    }
    return false;
}


bool ultimateTrackerThread::readFromTracker()
{
    if (templatePFTrackerBottle = templatePFTrackerTarget->read(false))
    {
        if (templatePFTrackerBottle!=NULL)
        {
            templatePFTrackerPos(0) = templatePFTrackerBottle->get(0).asDouble();
            templatePFTrackerPos(1) = templatePFTrackerBottle->get(1).asDouble();
        }
    }

    if (noInput())
    {
        stateFlag=0;
    }
    return false;
}

bool ultimateTrackerThread::getPointFromStereo()
{
    Bottle cmdSFM;
    Bottle respSFM;
    cmdSFM.clear();
    respSFM.clear();
    cmdSFM.addString("Point");
    cmdSFM.addInt(int(templatePFTrackerPos(0)));
    cmdSFM.addInt(int(templatePFTrackerPos(1)));
    SFMrpcPort.write(cmdSFM, respSFM);

    // Read the 3D coords and compute the distance to the set reference frame origin
    if (respSFM.size() == 3)
    {
        SFMPos(0) = respSFM.get(0).asDouble(); // Get the X coordinate
        SFMPos(1) = respSFM.get(1).asDouble(); // Get the Y coordinate
        SFMPos(2) = respSFM.get(2).asDouble(); // Get the Z coordinate
        return true;
    } 

    return false;
}

bool ultimateTrackerThread::initializeTracker()
{
    Network::connect("/motionCUT/crop:o","/templatePFTracker/template/image:i");
    yarp::os::Time::delay(0.25);
    Network::disconnect("/motionCUT/crop:o","/templatePFTracker/template/image:i");
    return false;
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
    printMessage(0,"Closing ports..\n");
        closePort(motionCUTBlobs);
        printMessage(1,"    motionCUTBlobs successfully closed!\n");
}

// empty line to make gcc happy
