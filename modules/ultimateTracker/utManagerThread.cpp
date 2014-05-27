#include "utManagerThread.h"

utManagerThread::utManagerThread(int _rate, const string &_name, const string &_robot, int _v, kalmanThread *_kT) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    kalThrd   = _kT;
    stateFlag = 0;
    timeNow   = yarp::os::Time::now();

    motionCUTBlobs = new BufferedPort<Bottle>;
    motionCUTPos.resize(2,0.0);
    
    templatePFTrackerTarget = new BufferedPort<Bottle>;
    templatePFTrackerPos.resize(2,0.0);

    SFMPos.resize(3,0.0);
}

bool utManagerThread::threadInit()
{
    motionCUTBlobs -> open(("/"+name+"/mCUT:i").c_str());
    templatePFTrackerTarget -> open(("/"+name+"/pfTracker:i").c_str());
    SFMrpcPort.open(("/"+name+"/SFM:o").c_str());
    outportGui.open(("/"+name+"/gui:o").c_str());

    Network::connect("/motionCUT/blobs:o",("/"+name+"/mCUT:i").c_str());
    Network::connect("/templatePFTracker/target:o",("/"+name+"/pfTracker:i").c_str());
    Network::connect(("/"+name+"/SFM:o").c_str(),"/SFM/rpc");
    Network::connect(("/"+name+"/gui:o").c_str(),"/iCubGui/objects");

    return true;
}

void utManagerThread::run()
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
            readFromTracker();
            if (getPointFromStereo())
            {
                printMessage(0,"Initializing Kalman filter...\n");
                stateFlag++;
            }
            break;
        case 3:
            printMessage(2,"Reading from tracker and SFM...\n");
            readFromTracker();
            if (getPointFromStereo())
            {
                manageiCubGui();
            }
            break;
        default:
            printMessage(0,"ERROR!!! utManagerThread should never be here!!!\nState: %d\n",stateFlag);
            Time::delay(1);
            break;
    }
    printMessage(1,"stateFlag %i kalmanPos: \n",stateFlag);
}

bool utManagerThread::manageKalman()
{
    return true;
}

bool utManagerThread::manageiCubGui()
{
    if (outportGui.getOutputCount()>0)
    {
        Bottle obj;
        obj.addString("object");
        obj.addString("MONSTER");
     
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

bool utManagerThread::noInput()
{
    if (yarp::os::Time::now() - timeNow > 1.0)
    {
        timeNow = yarp::os::Time::now();
        return true;
    }
    return false;
}

bool utManagerThread::processMotion()
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

bool utManagerThread::stabilityCheck()
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

bool utManagerThread::readFromTracker()
{
    if (templatePFTrackerBottle = templatePFTrackerTarget->read(false))
    {
        if (templatePFTrackerBottle!=NULL)
        {
            timeNow = yarp::os::Time::now();
            templatePFTrackerPos(0) = templatePFTrackerBottle->get(0).asDouble();
            templatePFTrackerPos(1) = templatePFTrackerBottle->get(1).asDouble();
        }
    }

    if (noInput())
    {
        stateFlag = 0;
    }
    return false;
}

bool utManagerThread::getPointFromStereo()
{
    Bottle cmdSFM;
    Bottle respSFM;
    cmdSFM.clear();
    respSFM.clear();
    cmdSFM.addString("Root");
    cmdSFM.addInt(int(templatePFTrackerPos(0)));
    cmdSFM.addInt(int(templatePFTrackerPos(1)));
    SFMrpcPort.write(cmdSFM, respSFM);

    // Read the 3D coords and compute the distance to the set reference frame origin
    if (respSFM.size() == 3)
    {
        Vector SFMtmp(3,0.0);
        SFMtmp(0) = respSFM.get(0).asDouble(); // Get the X coordinate
        SFMtmp(1) = respSFM.get(1).asDouble(); // Get the Y coordinate
        SFMtmp(2) = respSFM.get(2).asDouble(); // Get the Z coordinate

        if (SFMtmp(0) == 0.0 && SFMtmp(1) == 0.0 && SFMtmp(2) == 0.0)
        {
            return false;
        }

        SFMPos = SFMtmp;
        return true;
    } 

    return false;
}

bool utManagerThread::initializeTracker()
{
    Network::connect("/motionCUT/crop:o","/templatePFTracker/template/image:i");
    yarp::os::Time::delay(0.25);
    Network::disconnect("/motionCUT/crop:o","/templatePFTracker/template/image:i");
    return false;
}

int utManagerThread::printMessage(const int l, const char *f, ...) const
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

void utManagerThread::threadRelease()
{
    printMessage(0,"Closing ports..\n");
        closePort(motionCUTBlobs);
        printMessage(1,"    motionCUTBlobs successfully closed!\n");
}

// empty line to make gcc happy
