#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "ultimateTrackerThread.h"

#define KALMAN_INIT     0
#define KALMAN_NORMAL   1
#define KALMAN_NOINPUT  2
#define KALMAN_NEWDATA  3

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

    kalState  = KALMAN_INIT;
    kalOrder  = 4;
    kalEst.resize(kalOrder,0.0);
    kalStateVec.resize(kalOrder,0.0);
    kalTs     = 0.1;
    kalClock  = 0.0;
    kalA      = eye(kalOrder);
    kalA(0,1) = 0.01;
    kalA(0,2) = 0.0001;
    kalA(1,2) = 0.01;
    kalA(1,3) = 0.0001;
    kalA(2,3) = 0.01;

    kalH      = zeros(1,kalOrder);
    kalH(0,0) = 1;

    kalQ = 0.00001 * eye(kalOrder);
    kalP = 0.00001 * eye(kalOrder);

    // Threshold is set to chi2inv(0.95,1)
    kalThres = 3.8415;

    kalR      = eye(1);
    kalR(0,0) = 0.01;

    for (int i = 0; i < 3; i++)
    {
        posVelKalman.push_back(Kalman(kalA,kalH,kalQ,kalR));
    }
}

bool ultimateTrackerThread::threadInit()
{
    motionCUTBlobs -> open(("/"+name+"/mCUT:i").c_str());
    templatePFTrackerTarget -> open(("/"+name+"/pfTracker:i").c_str());
    SFMrpcPort.open(("/"+name+"/SFM:o").c_str());
    outportGui.open(("/"+name+"/gui:o").c_str());

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
            kalClock=0.0;
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
                manageKalman();
                stateFlag++;
            }
            break;
        case 3:
            printMessage(2,"Reading from tracker and SFM...\n");
            readFromTracker();
            if (getPointFromStereo())
            {
                kalState == KALMAN_NEWDATA;
                manageKalman();
                manageiCubGui();
            }
            else
                manageKalman();
            break;
        default:
            printMessage(0,"ERROR!!! ultimateTrackerThread should never be here!!!\nState: %d\n",stateFlag);
            Time::delay(1);
            break;
    }
    kalClock += kalTs;
}

Vector ultimateTrackerThread::manageKalman()
{
    Vector res(3,-1.0);
    // If init is equal to 0, the filter has to be initialized
    if (kalState == KALMAN_INIT)
    {
        for (int i = 0; i < 3; i++)
        {
            Vector state0(1,SFMPos(i));
            kalClock = 0.0;
            posVelKalman[i].init(state0,kalP);
            // currEst  = posVelKalman(i).filt(SFMPos(i));
            // res(i)   = currEst(0);
            // res(2+i) = currEst(1);
        }
    }
    // If init==1 (i.e. I've got a new data) and kalClock>=kalTs (i.e.
    // the filter did a new step), let's correct the current estimation
    // with the new measurement
    else if (kalState==KALMAN_NEWDATA)
    {
        for (int i = 0; i < 3; i++)
        {
            Vector measurement(1,SFMPos(i));
            posVelKalman[i].correct(measurement);

            // After correction, let's check if the validationGate has overcome the threshold.
            // If this is the case, let's go back to the initial state.
            if ( posVelKalman[i].get_ValidationGate() > kalThres )
            {
                printMessage(0,"Validation Gate for kalman #%i has been overcome! Returning to initial state.",i);
                stateFlag = 0;
                kalState == KALMAN_INIT;
                return res;
            }
        }
        kalState == KALMAN_NOINPUT;
    }

    if (kalState != KALMAN_INIT)
    {
        Vector prediction;
        for (int i = 0; i < 3; i++)
        {
            Vector input(1,0.0);
            prediction = posVelKalman[i].predict(input);
            res(i) = prediction(0);
        }
    }

    return res;
}

bool ultimateTrackerThread::manageiCubGui()
{
    if (outportGui.getOutputCount()>0)
    {
        Bottle obj;
        obj.addString("object");
        obj.addString("ball");
     
        // size 
        obj.addDouble(50.0);
        obj.addDouble(50.0);
        obj.addDouble(50.0);
    
        // positions
        obj.addDouble(1000.0*SFMPos[0]);
        obj.addDouble(1000.0*SFMPos[1]);
        obj.addDouble(1000.0*SFMPos[2]);
    
        // orientation
        obj.addDouble(0.0);
        obj.addDouble(0.0);
        obj.addDouble(0.0);
    
        // color
        obj.addInt(255);
        obj.addInt(125);
        obj.addInt(125);
    
        // transparency
        obj.addDouble(0.9);
    
        outportGui.write(obj);
    }
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
