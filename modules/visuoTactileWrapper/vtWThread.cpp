#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtWThread.h"

vtWThread::vtWThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    optFlowPos.resize(3,0.0);
    optFlowVelEstimate.resize(3,0.0);

    pf3dTrackerPos.resize(3,0.0);
    pf3dTrackerVelEstimate.resize(3,0.0);

    doubleTouchPos.resize(3,0.0);
    doubleTouchVelEstimate.resize(3,0.0);

    fgtTrackerPos.resize(3,0.0);
    fgtTrackerVelEstimate.resize(3,0.0);
    
    armR = new iCubArm("right");
    armL = new iCubArm("left");

    doubleTouchStep =   -1;
}

bool vtWThread::threadInit()
{
    optFlowPort.open(("/"+name+"/optFlow:i").c_str());
    pf3dTrackerPort.open(("/"+name+"/pf3dTracker:i").c_str());
    doubleTouchPort.open(("/"+name+"/doubleTouch:i").c_str());
    fgtTrackerPort.open(("/"+name+"/fingertipTracker:i").c_str());
    eventsPort.open(("/"+name+"/events:o").c_str());
    depth2kinPort.open(("/"+name+"/depth2kin:o").c_str());
    
    Network::connect("/ultimateTracker/Manager/events:o",("/"+name+"/optFlow:i").c_str());
    Network::connect("/pf3dTracker/data:o",("/"+name+"/pf3dTracker:i").c_str());
    Network::connect("/doubleTouch/status:o",("/"+name+"/doubleTouch:i").c_str());
    Network::connect("/fingertipTracker/out:o",("/"+name+"/fingertipTracker:i").c_str());
    Network::connect(("/"+name+"/events:o").c_str(),"/visuoTactileRF/events:i");

    Property OptGaze;
    OptGaze.put("device","gazecontrollerclient");
    OptGaze.put("remote","/iKinGazeCtrl");
    OptGaze.put("local",("/"+name+"/gaze").c_str());

    if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
       yError(" could not open the Gaze Controller!");
       return false;
    }

    igaze -> storeContext(&contextGaze);
    igaze -> setSaccadesStatus(false);
    igaze -> setNeckTrajTime(0.75);
    igaze -> setEyesTrajTime(0.5);
    
    // /**************************/
    //     Property OptR;
    //     OptR.put("robot",  robot.c_str());
    //     OptR.put("part",   "right_arm");
    //     OptR.put("device", "remote_controlboard");
    //     OptR.put("remote",("/"+robot+"/right_arm").c_str());
    //     OptR.put("local", ("/"+name +"/right_arm").c_str());

    //     if (!ddR.open(OptR))
    //     {
    //         yError(" could not open right_arm PolyDriver!");
    //         return false;
    //     }
    //     bool ok = 1;
    //     if (ddR.isValid())
    //     {
    //         ok = ok && ddR.view(iencsR);
    //     }
    //     if (!ok)
    //     {
    //         yError(" Problems acquiring right_arm interfaces!!!!");
    //         return false;
    //     }
    //     iencsR->getAxes(&jntsR);
    //     encsR = new yarp::sig::Vector(jntsR,0.0);

    /**************************/
        Property OptL;
        OptL.put("robot",  robot.c_str());
        OptL.put("part",   "left_arm");
        OptL.put("device", "remote_controlboard");
        OptL.put("remote",("/"+robot+"/left_arm").c_str());
        OptL.put("local", ("/"+name +"/left_arm").c_str());

        if (!ddL.open(OptL))
        {
            yError(" could not open left_arm PolyDriver!");
            return false;
        }
        bool ok = 1;
        if (ddL.isValid())
        {
            ok = ok && ddL.view(iencsL);
        }
        if (!ok)
        {
            yError(" Problems acquiring left_arm interfaces!!!!");
            return false;
        }
        iencsL->getAxes(&jntsL);
        encsL = new yarp::sig::Vector(jntsL,0.0);

    linEst_optFlow     = new AWLinEstimator(16,0.05);
    linEst_pf3dTracker = new AWLinEstimator(16,0.05);
    linEst_doubleTouch = new AWLinEstimator(16,0.05);
    linEst_fgtTracker  = new AWLinEstimator(16,0.05);

    timeNow = yarp::os::Time::now();
    
    return true;
}

void vtWThread::run()
{
    bool isTarget = false;
    events.clear();

    // process the optFlow
    if (optFlowBottle = optFlowPort.read(false))
    {
        if (optFlowBottle->size()>=3)
        {
            optFlowPos.zero();
            
            optFlowPos[0]=optFlowBottle->get(0).asDouble();
            optFlowPos[1]=optFlowBottle->get(1).asDouble();
            optFlowPos[2]=optFlowBottle->get(2).asDouble();

            AWPolyElement el(optFlowPos,Time::now());
            optFlowVelEstimate=linEst_optFlow->estimate(el);

            events.push_back(IncomingEvent(optFlowPos,optFlowVelEstimate,0.05,"optFlow"));
            isTarget=true;
        }
    }

    // process the pf3dTracker
    if (pf3dTrackerBottle = pf3dTrackerPort.read(false))
    {
        if (pf3dTrackerBottle->size()>6)
        {
            if (pf3dTrackerBottle->get(6).asDouble()==1.0)
            {
                Vector fp(4);
                fp[0]=pf3dTrackerBottle->get(0).asDouble();
                fp[1]=pf3dTrackerBottle->get(1).asDouble();
                fp[2]=pf3dTrackerBottle->get(2).asDouble();
                fp[3]=1.0;

                if (!gsl_isnan(fp[0]) && !gsl_isnan(fp[1]) && !gsl_isnan(fp[2]))
                {
                    yDebug("Computing data from the pf3dTracker %g\n",getEstUsed());
                    Vector x,o;
                    igaze->getLeftEyePose(x,o);
                    
                    Matrix T=axis2dcm(o);
                    T(0,3)=x[0];
                    T(1,3)=x[1];
                    T(2,3)=x[2];

                    pf3dTrackerPos=T*fp;
                    pf3dTrackerPos.pop_back();
                    
                    AWPolyElement el(pf3dTrackerPos,Time::now());
                    pf3dTrackerVelEstimate=linEst_pf3dTracker->estimate(el);
                    
                    events.push_back(IncomingEvent(pf3dTrackerPos,pf3dTrackerVelEstimate,0.05,"pf3dTracker"));
                    isTarget=true;
                }
            }
        }
    }

    // process the fingertipTracker
    if(fgtTrackerBottle = fgtTrackerPort.read(false))
    {
        if (doubleTouchBottle = doubleTouchPort.read(false))
        {           
            if(fgtTrackerBottle != NULL && doubleTouchBottle != NULL)
            {
                if (doubleTouchBottle->get(3).asString() != "" && fgtTrackerBottle->get(0).asInt() != 0)
                {
                    doubleTouchStep = doubleTouchBottle->get(0).asInt();
                    fgtTrackerPos[0] = fgtTrackerBottle->get(1).asDouble();
                    fgtTrackerPos[1] = fgtTrackerBottle->get(2).asDouble();
                    fgtTrackerPos[2] = fgtTrackerBottle->get(3).asDouble();
                    AWPolyElement el2(fgtTrackerPos,Time::now());
                    fgtTrackerVelEstimate=linEst_fgtTracker->estimate(el2);

                    if(doubleTouchStep<=1)
                    {
                        Vector ang(3,0.0);
                        igaze -> lookAtAbsAngles(ang);
                    }
                    else if(doubleTouchStep>1 && doubleTouchStep<8)
                    {
                        events.clear();
                        events.push_back(IncomingEvent(fgtTrackerPos,fgtTrackerVelEstimate,-1.0,"fingertipTracker"));
                        isTarget=true;
                    }
                }
            }
        }
    }

    // process the doubleTouch
    if(doubleTouchBottle = doubleTouchPort.read(false))
    {
        if(doubleTouchBottle != NULL)
        {
            if (doubleTouchBottle->get(3).asString() != "")
            {
                Matrix T = eye(4);
                Vector fingertipPos(4,0.0);
                doubleTouchPos.resize(4,0.0);
                
                currentTask = doubleTouchBottle->get(3).asString();
                doubleTouchStep = doubleTouchBottle->get(0).asInt();
                fingertipPos = matrixFromBottle(*doubleTouchBottle,20,4,4).subcol(0,3,3); // fixed translation from the palm
                fingertipPos.push_back(1.0);

                if(doubleTouchStep<=1)
                {
                    Vector ang(3,0.0);
                    igaze -> lookAtAbsAngles(ang);
                }
                else if(doubleTouchStep>1 && doubleTouchStep<8)
                {
                    if(currentTask=="LtoR" || currentTask=="LHtoR") //right to left -> the right index finger will be generating events
                    { 
                        iencsR->getEncoders(encsR->data());
                        Vector qR=encsR->subVector(0,6);
                        armR -> setAng(qR*CTRL_DEG2RAD);                        
                        T = armR -> getH(3+6, true);  // torso + up to wrist
                        doubleTouchPos = T * fingertipPos; 
                        //optionally, get the finger encoders and get the fingertip position using iKin Finger based on the current joint values 
                        //http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html#sec_cart_tipframe
                        doubleTouchPos.pop_back(); //take out the last dummy value from homogenous form
                    }
                    else if(currentTask=="RtoL" || currentTask=="RHtoL") //left to right -> the left index finger will be generating events
                    {   
                        iencsL->getEncoders(encsL->data());
                        Vector qL=encsL->subVector(0,6);
                        armL -> setAng(qL*CTRL_DEG2RAD);                        
                        T = armL -> getH(3+6, true);  // torso + up to wrist
                        doubleTouchPos = T * fingertipPos; 
                        //optionally, get the finger encoders and get the fingertip position using iKin Finger based on the current joint values 
                        //http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html#sec_cart_tipframe
                        doubleTouchPos.pop_back(); //take out the last dummy value from homogenous form
                    } 
                    else
                    {
                        yError(" vtWThread::run(): Unknown task received from double touch thread!");
                    }
                                   
                    AWPolyElement el2(doubleTouchPos,Time::now());
                    doubleTouchVelEstimate=linEst_doubleTouch->estimate(el2);
                    events.push_back(IncomingEvent(doubleTouchPos,doubleTouchVelEstimate,-1.0,"doubleTouch"));
                    isTarget=true;
                }
            }
        }
    }
    
    if (pf3dTrackerPos[0]!=0.0 && pf3dTrackerPos[1]!=0.0 && pf3dTrackerPos[2]!=0.0)
        igaze -> lookAtFixationPoint(pf3dTrackerPos);
    else if (doubleTouchPos[0]!=0.0 && doubleTouchPos[1]!=0.0 && doubleTouchPos[2]!=0.0)
        igaze -> lookAtFixationPoint(doubleTouchPos);
    else if (optFlowPos[0]!=0.0 && optFlowPos[1]!=0.0 && optFlowPos[2]!=0.0)
        igaze -> lookAtFixationPoint(optFlowPos);
    
    if (isTarget)
    {        
        Bottle& eventsBottle = eventsPort.prepare();
        eventsBottle.clear();
        for (size_t i = 0; i < events.size(); i++)
        {
            eventsBottle.addList()= events[i].toBottle();
        }
        eventsPort.write();
        timeNow = yarp::os::Time::now();
    }
    else if (yarp::os::Time::now() - timeNow > 1.0)
    {
        yDebug("No significant event in the last second. Resetting the velocity estimators..");
        timeNow = yarp::os::Time::now();

        linEst_optFlow     -> reset();
        linEst_pf3dTracker -> reset();
        linEst_doubleTouch -> reset();
        linEst_fgtTracker  -> reset();
    }
}

int vtWThread::printMessage(const int l, const char *f, ...) const
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

void vtWThread::threadRelease()
{
    yDebug("Closing gaze controller..");
        Vector ang(3,0.0);
        igaze -> lookAtAbsAngles(ang);
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();

    yDebug("Closing estimators..");
        delete linEst_optFlow;
        linEst_optFlow = NULL;
        
        delete linEst_pf3dTracker;
        linEst_pf3dTracker = NULL;

        delete linEst_doubleTouch;
        linEst_doubleTouch = NULL;

        delete linEst_fgtTracker;
        linEst_fgtTracker = NULL;

    yDebug("Closing ports..");
        optFlowPort.interrupt();
        optFlowPort.close();
        yTrace("optFlowPort successfully closed!");
        pf3dTrackerPort.interrupt();
        pf3dTrackerPort.close();
        yTrace("pf3dTrackerPort successfully closed!");
        doubleTouchPort.interrupt();
        doubleTouchPort.close();
        yTrace("doubleTouchPort successfully closed!");
        eventsPort.interrupt();
        eventsPort.close();
        yTrace("eventsPort successfully closed!");
        depth2kinPort.interrupt();
        depth2kinPort.close();
        yTrace("depth2kinPort successfully closed!");
}

// empty line to make gcc happy
