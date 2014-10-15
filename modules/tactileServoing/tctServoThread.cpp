#include "tctServoThread.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5
#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

tactileServoThread::tactileServoThread(int _rate, const string &_name, const string &_robot, int _v) :
                                     RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    armR = new iCubArm("right");
    armL = new iCubArm("left");
}

bool tactileServoThread::threadInit()
{
    Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name +"/right_arm").c_str());

    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name +"/left_arm").c_str());

    // if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
    //    printMessage(0,"Error: could not open the Gaze Controller!\n");
    //    return false;
    // }

    // igaze -> storeContext(&contextGaze);
    // if (robot == "icubSim")
    // {
    //     igaze -> setNeckTrajTime(1.5);
    //     igaze -> setEyesTrajTime(1.0);
    // }
    // //else
    // //{
    // //    igaze -> setNeckTrajTime(1.5);
    // //    igaze -> setEyesTrajTime(0.5);
    // //}
    // igaze -> setSaccadesStatus(false);

    if (!ddR.open(OptR))
    {
        printMessage(0,"ERROR: could not open right_arm PolyDriver!\n");
        return false;
    }
    if (!ddL.open(OptL))
    {
        printMessage(0,"ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }

    bool ok = 1;

    if (ddL.isValid())
    {
        ok = ok && ddL.view(iencsL);
        ok = ok && ddL.view(iposL);
        ok = ok && ddL.view(imodeL);
        ok = ok && ddL.view(iimpL);
    }
    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(iposR);
        ok = ok && ddR.view(imodeR);
        ok = ok && ddR.view(iimpR);
    }
    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    // Vector joints;
    // iencsM->getEncoders(encsM->data());
    // slv->probl->index.getChainJoints(*encsM,joints);
    // Matrix HIndex=slv->probl->index.getH(joints*CTRL_DEG2RAD);
    // slv->probl->limb.setHN(HIndex);
    // testLimb->setHN(HIndex);
    // printMessage(1,"HIndex:\n%s\n", HIndex.toString().c_str());

    return true;
}

void tactileServoThread::run()
{

}

int tactileServoThread::printMessage(const int l, const char *f, ...) const
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

void tactileServoThread::threadRelease()
{
    printMessage(0,"Deleting arms..\n");
        if (armL)
        {
            delete armL;
            armL = NULL;
        }
        if (armR)
        {
            delete armR;
            armR = NULL;
        }

}

// empty line to make gcc happy
