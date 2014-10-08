#include "iCubDblTchThrd.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5
#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

doubleTouchThread::doubleTouchThread(int _rate, const string &_name, const string &_robot, int _v,
                                     string _type, int _record, string _filename, string _color) :
                                     RateThread(_rate), name(_name), robot(_robot), verbosity(_v),
                                     type(_type), record(_record), filename(_filename), color(_color)
{
    step     = 0;
    recFlag  = 0;
    skinPort = new BufferedPort<iCub::skinDynLib::skinContactList>;
    outPort  = new BufferedPort<Bottle>;
    slv      = new doubleTouch_Solver(type);

    gue   = new doubleTouch_Variables(slv->probl->getNVars()); // guess
    sol   = new doubleTouch_Variables(slv->probl->getNVars()); // solution
    solution.resize(slv->probl->getNVars(),0.0);

    armR = new iCubArm("right");
    armL = new iCubArm("left");

    gue->joints[1] = -45.0*CTRL_DEG2RAD; gue->joints[3] = -30.0*CTRL_DEG2RAD;
    gue->joints[4] =  30.0*CTRL_DEG2RAD; gue->joints[5] = -30.0*CTRL_DEG2RAD;
    gue->joints[6] =  30.0*CTRL_DEG2RAD; gue->joints[8] =  45.0*CTRL_DEG2RAD;
    sol->clone(*gue);

    slv->probl->limb.setAng(gue->joints);

    testLimb = new iCubCustomLimb(type);
    skinPart = -1;

    if (type == "LtoR")
    {
        skinPart = FOREARM_LEFT;
    }
    else if (type == "LHtoR")
    {
        skinPart = HAND_LEFT;
    }
    else if (type == "RtoL")
    {
        skinPart = FOREARM_RIGHT;
    }
    else if (type == "RHtoL")
    {
        skinPart = HAND_RIGHT;
    }

    contextGaze = -1;
    iter = 1;

    cntctPosLink.resize(3,0.0);
    cntctPosWRF.resize(3,0.0);
    cntctPosEE.resize(3,0.0);
    cntctNormDir.resize(3,0.0);
    cntctPressure = -1;
    cntctLinkNum  = -1;
    cntctSkinPart = "";
    cntctH0 = eye(4);

    oldEEL.resize(3,0.0);
    oldEER.resize(3,0.0);
}

bool doubleTouchThread::threadInit()
{
    skinPort -> open(("/"+name+"/contacts:i").c_str());
    outPort  -> open(("/"+name+"/status:o").c_str());

    if (robot=="icubSim")
    {
        Network::connect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str());
    }
    Network::connect(("/"+name+"/status:o").c_str(),"/visuoTactileRF/input:i");


    // Property OptGaze;
    // OptGaze.put("device","gazecontrollerclient");
    // OptGaze.put("remote","/iKinGazeCtrl");
    // OptGaze.put("local",("/"+name+"/gaze").c_str());

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

    // open the view
    bool ok = 1;
    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(iposR);
        ok = ok && ddR.view(ictrlR);
        ok = ok && ddR.view(iimpR);
        ok = ok && ddR.view(ilimR);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring right_arm interfaces!!!!\n");
        return false;
    }

    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    if (!ddL.open(OptL))
    {
        printMessage(0,"ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }

    // open the view
    ok = 1;
    if (ddL.isValid())
    {
        ok = ok && ddL.view(iencsL);
        ok = ok && ddL.view(iposL);
        ok = ok && ddL.view(ictrlL);
        ok = ok && ddL.view(iimpL);
        ok = ok && ddL.view(ilimL);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring left_arm interfaces!!!!\n");
        return false;
    }

    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    if (!alignJointsBounds())
    {
        printMessage(0,"\nERROR: alignJointsBounds failed!!!\n");
        return false;
    }
    
    ok = 1;
    ok = ok && iimpR->setImpedance(0,  0.4, 0.03);
    ok = ok && iimpR->setImpedance(1, 0.35, 0.03);
    ok = ok && iimpR->setImpedance(2, 0.35, 0.03);
    ok = ok && iimpR->setImpedance(3,  0.2, 0.02);
    ok = ok && iimpR->setImpedance(4,  0.2, 0.00);
    
    ok = ok && iimpL->setImpedance(0,  0.4, 0.03);
    ok = ok && iimpL->setImpedance(1, 0.35, 0.03);
    ok = ok && iimpL->setImpedance(2, 0.35, 0.03);
    ok = ok && iimpL->setImpedance(3,  0.2, 0.02);
    ok = ok && iimpL->setImpedance(4,  0.2, 0.00);

    Vector joints;
    if ((type == "LtoR") || (type == "LHtoR"))
    {
        iencsR->getEncoders(encsR->data());
        slv->probl->index.getChainJoints(*encsR,joints);
    }
    if ((type == "RtoL") || (type == "RHtoL"))
    {
        iencsL->getEncoders(encsL->data());
        slv->probl->index.getChainJoints(*encsL,joints);
    }

    Matrix HIndex=slv->probl->index.getH(joints*CTRL_DEG2RAD);
    slv->probl->limb.setHN(HIndex);
    testLimb->setHN(HIndex);
    printMessage(1,"HIndex\n%s\n", HIndex.toString().c_str());

    return true;
}

void doubleTouchThread::run()
{
    skinContactList *skinContacts  = skinPort -> read(false);

    Bottle& output = outPort->prepare();
    output.clear();
    output.addInt(step);
    output.addInt(record);
    output.addInt(recFlag);
    output.addString(slv->probl->getType().c_str());
    matrixIntoBottle(cntctH0,output);
    matrixIntoBottle(slv->probl->limb.getHN(),output);
    matrixIntoBottle(armM->getH(),output);
    outPort->write();

    // handleGaze();

    if (checkMotionDone())
    {
        switch (step)
        {
            case 0:
                printMessage(1,"Moving to rest...\n");
                goToRest();
                // move the thumbs close to the hand
                if ((type == "LtoR") || (type == "both"))
                {
                    //iposR -> positionMove(8,10);    iposR -> positionMove(9,90);
                    //iposR -> positionMove(13,90);   iposR -> positionMove(14,180);   iposR -> positionMove(15,270);
                    // drive the hand in pointing pose
                    Vector poss(9,0.0);
                    Vector vels(9,0.0);

                    poss[0]=40.0;  vels[0]=100.0;                     poss[1]=10.0;  vels[1]=100.0;
                    poss[2]=60.0;  vels[2]=100.0;                     poss[3]=70.0;  vels[3]=100.0;
                    poss[4]=00.0;  vels[4]=100.0;                     poss[5]=00.0;  vels[5]=100.0;
                    poss[6]=70.0;  vels[6]=100.0;                     poss[7]=100.0; vels[7]=100.0;
                    poss[8]=240.0; vels[8]=200.0; 
                    printf("configuring right hand...\n");
                    for (int i=7; i<jntsR; i++)
                    {
                        iposR->setRefAcceleration(i,1e9);
                        iposR->setRefSpeed(i,vels[i-7]);
                        iposR->positionMove(i,poss[i-7]);
                    }

                    poss[1]=10.0;  vels[1]=100.0;                     poss[2]=60.0;  vels[2]=100.0;
                    poss[3]=70.0;  vels[3]=100.0;                     poss[4]=00.0;  vels[4]=100.0;
                    poss[5]=00.0;  vels[5]=100.0;                     poss[6]=00.0;  vels[6]=100.0;
                    poss[7]=00.0;  vels[7]=100.0;                     poss[8]=00.0;  vels[8]=200.0;

                    printf("configuring left hand...\n");
                    for (int i=7; i<jntsL; i++)
                    {
                        iposL->setRefAcceleration(i,1e9);
                        iposL->setRefSpeed(i,vels[i-7]);
                        iposL->positionMove(i,poss[i-7]);
                    }

                    if (robot == "icubSim")
                    {
                        vels.resize(7,4.0);
                        for (int i=0; i<7; i++)
                            iposL->setRefSpeed(i,vels[i]);
                    }
                    else if (robot == "icub")
                    {
                        vels.resize(7,16.0);
                        for (int i=0; i<7; i++)
                            iposL->setRefSpeed(i,vels[i]);
                        for (int i=0; i<7; i++)
                            iposR->setRefSpeed(i,vels[i]);
                    }
                }
                if ((type == "RtoL") || (type == "both"))
                {
                    //iposL -> positionMove(8,10);    iposL -> positionMove(9,90);
                    //iposL -> positionMove(13,90);   iposL -> positionMove(14,180);   iposL -> positionMove(15,270);
                }
                printMessage(0,"*************\nWAITING FOR CONTACT...\n");
                step++;
                break;
            case 1:
                if(skinContacts)
                {
                    printMessage(3,"Waiting for contact..\n");
                    detectContact(skinContacts); // READ A CONTACT ON THE SKIN
                    if (cntctSkinPart != "")
                    {
                        // printMessage(0,"CONTACT!!! skinContact: %s\nskinPart: %s Link: %i Position: %s NormDir: %s\n", cntctSkin.toString().c_str(),
                        //             cntctSkinPart.c_str(), cntctLinkNum,cntctPosLink.toString().c_str(),cntctNormDir.toString().c_str());
                        printMessage(0,"CONTACT!!! skinPart: %s Link: %i Position: %s NormDir: %s\n",
                                        cntctSkinPart.c_str(), cntctLinkNum,cntctPosLink.toString().c_str(),
                                        cntctNormDir.toString().c_str());

                        printMessage(1,"Switching to impedance position mode..\n");
                        ictrlS -> setImpedancePositionMode(2);
                        ictrlS -> setImpedancePositionMode(3);
                        step++;
                    }
                }
                break;
            case 2:
                solveIK();
                printMessage(0,"Going to taxel... Desired EE: %s\n",(sol->ee).toString().c_str());
                printMessage(2,"jnts=%s\n",(sol->joints*CTRL_RAD2DEG).toString().c_str());
                step++;
                recFlag = 1;
                break;
            case 3:
                //if (record != 0)
                //{
                    delay(2);
                //}
                if (record == 0)
                {
                    goToTaxel();
                    step += 3;
                }
                else
                {
                    goToTaxelMaster();
                    step++;
                }
                break;
            case 4:
                if (record == 0)
                {
                    delay(1);
                }
                step++;
                break;
            case 5:
                goToTaxelSlave();
                step++;
                break;
            case 6:
                recFlag = 0;
                delay(1);
                
                bool flag;
                if (record == 0)
                {
                    // igaze->checkMotionDone(&flag);
                    Time::delay(2);
                    flag=1;
                    if (flag == 1)
                    {
                        testAchievement();
                        step += 2;
                    }
                }
                else
                {
                    testAchievement();
                    printMessage(0,"Waiting for the event to go back.\n");
                    step++;
                }
                break;
            case 7:
                if(skinContacts)
                {
                    if (record == 1)
                    {
                        if (testAchievement2(skinContacts))
                            step++;
                        else if (robot == "icub" && exitFromDeadlock(skinContacts))
                            step++;
                    }
                    else if (record == 2)
                    {
                        if (exitFromDeadlock(skinContacts))
                            step++;
                        else
                            testAchievement2(skinContacts);
                    }
                }
                break;
            case 8:
                printMessage(0,"Going to rest...\n");
                goToRest();
                printMessage(1,"Switching to position mode..\n");
                ictrlS -> setPositionMode(2);
                ictrlS -> setPositionMode(3);
                goToRest();

                printMessage(0,"*************\nWAITING FOR CONTACT...\n");
                step = 1;
                break;
            default:
                printMessage(0,"ERROR!!! doubleTouchThread should never be here!!!\nStep: %d",step);
                delay(1);
                break;
        }
    }
}

bool doubleTouchThread::exitFromDeadlock(skinContactList *_sCL)
{
    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if(it -> getSkinPart() == 7 && it -> getPressure() > 25)
        {
            // If the user touches the torso of the robot, it triggers
            // the return to the starting point
            printMessage(0,"I'm exiting from deadlock. Nothing is recorded! Iterator: %i\n",iter);
            return true;
        }
    }
    return false;
}

bool doubleTouchThread::testAchievement2(skinContactList *_sCL)
{
    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if(skinPart == it -> getSkinPart())
        {
            /**
            * ENCODERS SLAVE (They're 7 DOF straightforwardly acquired from shoulder to wrist)
            */
            iencsS->getEncoders(encsS->data());
            Vector qS=encsS->subVector(0,12);
            armS -> setAng(qS*CTRL_DEG2RAD);

            /**
            * ENCODERS MASTER (They're 7 DOF straightforwardly acquired from shoulder to wrist)
            */
            iencsM->getEncoders(encsM->data());
            Vector qM=encsM->subVector(0,12);
            armM -> setAng(qM*CTRL_DEG2RAD);

            /**
            * FINAL TAXEL REFERENCE FRAME
            */
            Matrix cntctH0_final = findH0(*it);

            /**
            * WRITE ON FILE: ITERATOR, ABSOLUTE TIME, YARP TIMESTAMP
            * ROBOT, COLOR, TASKTYPE, ENCODERS SLAVE, ENCODERS MASTER, 
            * TARGET TAXEL REFERENCE FRAME, FINAL TAXEL REFERENCE FRAME,
            * INDEX HN
            */
            printMessage(0,"SUCCESS!!!! Self Touch Accomplished! Iterator: %i\n",iter);
            printMessage(1,"Encoders Slave:  %s\n", qS.toString().c_str());
            printMessage(1,"Encoders Master: %s\n", qM.toString().c_str());
            printMessage(0,"Target Position: %s\n", cntctH0.subcol(0,3,3).toString().c_str());
            printMessage(0,"Final  Position: %s\n", cntctH0_final.subcol(0,3,3).toString().c_str());

            ofstream outputfile;
            outputfile.open (filename.c_str(),ios::app);
            outputfile  << iter << "\t" << fixed << Time::now() << "\t"
                        << robot  << "\t" << color << "\t" << slv->probl->getType() << "\t"
                        << qS.toString() << "\t" << qM.toString() << "\t"
                        << toVector(cntctH0).toString() << "\t"
                        << toVector(cntctH0_final).toString();
            outputfile  << "\t" << toVector(slv->probl->limb.getHN()).toString();
            outputfile  << endl;
            outputfile.close();
            iter++;

            return true;
        }
    }
    return false;
}

bool doubleTouchThread::checkMotionDone()
{
    if (step == 1 || step == 7 || step == 8 || (record == 0 && (step == 4 || step == 5)))
        return true;
    
    iencsL->getEncoders(encsL->data());
    Vector qL=encsL->subVector(0,6);
    armL -> setAng(qL*CTRL_DEG2RAD);
    Vector eeL = armL -> EndEffPosition();

    iencsR->getEncoders(encsR->data());
    Vector qR=encsR->subVector(0,6);
    armR -> setAng(qR*CTRL_DEG2RAD);
    Vector eeR = armR -> EndEffPosition();

    double normR = norm(eeR - oldEER);
    double normL = norm(eeL - oldEEL);
    printMessage(1,"step: %i  result: %i  normR: %g\tnormL: %g\n", step,
        (normR <= VEL_THRES * getRate()) && (normL <= VEL_THRES * getRate()), normR, normL);

    oldEER = eeR;
    oldEEL = eeL;

    if ((normR <= VEL_THRES * getRate()) && (normL <= VEL_THRES * getRate())) {
        return true;
    }

    return false;
}

void doubleTouchThread::delay(int sec)
{
    for (int i = 0; i < sec*4; i++)
    {
        Time::delay(0.25);
        // if (!record)
        //     handleGaze();
    }
}

void doubleTouchThread::handleGaze()
{
    // if (step == 0 || step == 1 || step == 8) {
    //   Vector ang(3,0.0);
    //   igaze -> lookAtAbsAngles(ang);
    //   return true;
    // }
    // else if (record && step == 3)
    // {
    //     cntctPosWRF = findFinalConfiguration();
    //     printMessage(1,"cntctPosWRF: %s\n", cntctPosWRF.toString().c_str());
    //     igaze -> lookAtFixationPoint(cntctPosWRF);
    //     if (robot == "icub")
    //         igaze -> waitMotionDone();
    //     return true;
    // }
    // else if (record == 0)
    // {
    //     cntctPosWRF = locateContact(cntctSkin);
    //     igaze -> lookAtFixationPoint(cntctPosWRF);
    //     return true;
    // }
    // return false;
}

Vector doubleTouchThread::findFinalConfiguration()
{
    Vector q=solution.subVector(5,11);
    // cout << "q: " << q.toString() << endl;
    // cout << "q: " << (CTRL_RAD2DEG*(armM -> setAng(q*CTRL_DEG2RAD))).toString() << endl;
    armM -> setAng(q*CTRL_DEG2RAD);
    return armM -> EndEffPosition();
}

void doubleTouchThread::testAchievement()
{
    iencsM->getEncoders(encsM->data());
    iencsS->getEncoders(encsS->data());

    testLimb->setAng((*encsS)*CTRL_DEG2RAD,(*encsM)*CTRL_DEG2RAD);
    printMessage(0,"                 Final EE    %s\n", testLimb->EndEffPosition().toString().c_str());
    printMessage(2,"jnts=%s\n",(testLimb->getAng()*CTRL_RAD2DEG).toString().c_str());
}

void doubleTouchThread::solveIK(string s="standard")
{
    cntctH0 = findH0(cntctSkin);

    printMessage(2,"H0: \n%s\n",cntctH0.toString().c_str());
    slv->probl->limb.setH0(SE3inv(cntctH0));
    testLimb->setH0(SE3inv(cntctH0));

    slv->probl->limb.setAng(sol->joints);
    slv->setInitialGuess(*sol);
    slv->solve(*sol);
    // sol->print();
    solution = CTRL_RAD2DEG * sol->joints;

    testLimb->setAng(sol->joints);
}

void doubleTouchThread::goToTaxel()
{
    goToTaxelMaster();
    delay(1);
    goToTaxelSlave();
}

void doubleTouchThread::goToTaxelMaster()
{
    iposM -> positionMove(0,solution[5]);
    iposM -> positionMove(1,solution[6]);
    iposM -> positionMove(2,solution[7]);
    iposM -> positionMove(3,solution[8]);
    iposM -> positionMove(4,solution[9]);
    iposM -> positionMove(5,solution[10]);
    iposM -> positionMove(6,solution[11]);
}

void doubleTouchThread::goToTaxelSlave()
{
    iposS -> positionMove(4,-solution[0]);
    iposS -> positionMove(3,-solution[1]);
    iposS -> positionMove(2,-solution[2]);
    iposS -> positionMove(1,-solution[3]);
    iposS -> positionMove(0,-solution[4]);
}

void doubleTouchThread::goToRest()
{   
    Vector rest = CTRL_RAD2DEG * gue->joints;

    if ((slv->probl->getType() == "LtoR") || (slv->probl->getType() == "RtoL"))
    {
        iposS -> positionMove(4,-rest[0]);
        iposS -> positionMove(3,-rest[1]);
        iposS -> positionMove(2,-rest[2]);
        iposS -> positionMove(1,-rest[3]);
        iposS -> positionMove(0,-rest[4]);

        delay(3);

        iposM -> positionMove(0,rest[5]);
        iposM -> positionMove(1,rest[6]);
        iposM -> positionMove(2,rest[7]);
        iposM -> positionMove(3,rest[8]);
        iposM -> positionMove(4,rest[9]);
        iposM -> positionMove(5,rest[10]);
        iposM -> positionMove(6,rest[11]);
    }
    else
    {
        iposR -> positionMove(0,rest[5]);
        iposR -> positionMove(1,rest[6]);
        iposR -> positionMove(2,rest[7]);
        iposR -> positionMove(3,rest[8]);
        iposR -> positionMove(4,rest[9]);
        iposR -> positionMove(5,rest[10]);
        iposR -> positionMove(6,rest[11]);

        iposL -> positionMove(0,rest[5]);
        iposL -> positionMove(1,rest[6]);
        iposL -> positionMove(2,rest[7]);
        iposL -> positionMove(3,rest[8]);
        iposL -> positionMove(4,rest[9]);
        iposL -> positionMove(5,rest[10]);
        iposL -> positionMove(6,rest[11]);
    }
}

bool doubleTouchThread::alignJointsBounds()
{
    deque<IControlLimits*> lim;
    lim.push_back(ilimL);
    lim.push_back(ilimR);

    if (testLimb->       alignJointsBounds(lim) == 0) return false;
    if (slv->probl->limb.alignJointsBounds(lim) == 0) return false;

    if ((type == "LtoR") || (type == "LHtoR"))
    {
        lim.pop_front();
        if (slv->probl->index.alignJointsBounds(lim) == 0) return false;
    }
    if ((type == "RtoL") || (type == "RHtoL"))
    {
        lim.pop_back();
        if (slv->probl->index.alignJointsBounds(lim) == 0) return false;   
    } 
    return true;
}

void doubleTouchThread::detectContact(skinContactList *_sCL)
{
    // Reset variables:
    cntctPosLink.resize(3,0.0);
    cntctPosWRF.resize(3,0.0);
    cntctNormDir.resize(3,0.0);
    cntctPressure = -1;
    cntctLinkNum  = -1;
    cntctSkinPart = "";
    iposM  = NULL;          iposS  = NULL;
    encsM  = NULL;          encsS  = NULL;
    iencsM = NULL;          iencsS = NULL;
    ictrlM = NULL;          ictrlS = NULL;
    armM   = NULL;          armS   = NULL;

    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if( it -> getPressure() > 25 && skinPart == it -> getSkinPart())
        {
            cntctSkin = *it;                        // Store the skinContact for eventual future use
            cntctPosLink  = it -> getCoP();   // Get the position of the contact;
            cntctLinkNum  = it -> getLinkNumber();  // Retrieve the link number of the contact;
            cntctNormDir  = it -> getNormalDir();   // Normal direction of the contact
            cntctPressure = it -> getPressure();    // Retrieve the pressure of the contact

            if      (it -> getSkinPart() == FOREARM_LEFT)
            {
                cntctSkinPart = "forearm_left";
                iposM  = iposR;  iposS  = iposL;
                iencsM = iencsR; iencsS = iencsL;
                encsM  = encsR;  encsS  = encsL;
                ictrlM = ictrlR; ictrlS = ictrlL;
                armM   = armR;   armS   = armL;
            }
            else if (it -> getSkinPart() == FOREARM_RIGHT)
            {
                cntctSkinPart = "forearm_right";
                iposM  = iposL;  iposS  = iposR;
                iencsM = iencsL; iencsS = iencsR;
                encsM  = encsL;  encsS  = encsR;
                ictrlM = ictrlL; ictrlS = ictrlR;
                armM   = armL;   armS   = armR;
            }

            cntctPosWRF = locateContact(cntctSkin);
            printMessage(2,"cntctPosWRF: %s\n", cntctPosWRF.toString().c_str());
            cntctH0     = findH0(cntctSkin);
            break;
        }
    }
}

Vector doubleTouchThread::locateContact(skinContact &sc)
{
    Vector result(4,0.0);
    Matrix Twl = armS -> getH(cntctLinkNum+3, true);
    Vector posLink = cntctPosLink;
    posLink.push_back(1);
    result = Twl * posLink;
    result.pop_back();

    return result;
}

Matrix doubleTouchThread::findH0(skinContact &sc)
{
    // Set the proper orientation for the touching end-effector
    Matrix H0(4,4);
    Vector x(3,0.0), z(3,0.0), y(3,0.0);

    x = sc.getNormalDir();
    z[0] = -x[2]/x[0]; z[2] = 1;
    y = -1*(cross(x,z));
    
    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);

    H0.zero();
    H0(3,3) = 1;
    H0.setSubcol(x,0,0);
    H0.setSubcol(y,0,1);
    H0.setSubcol(z,0,2);
    H0.setSubcol(sc.getCoP(),0,3);
    return H0;
}

int doubleTouchThread::printMessage(const int l, const char *f, ...) const
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

void doubleTouchThread::threadRelease()
{
    printMessage(0,"Returning to position mode..\n");
        goToRest();
        // iposR -> positionMove(8,30);    iposR -> positionMove(9,0);
        // iposR -> positionMove(13,0);    iposR -> positionMove(14,0);   iposR -> positionMove(15,0);
        // iposL -> positionMove(8,30);    iposL -> positionMove(9,0);
        // iposL -> positionMove(13,0);    iposL -> positionMove(14,0);   iposL -> positionMove(15,0);

        ictrlR -> setPositionMode(2);
        ictrlR -> setPositionMode(3);
        ictrlL -> setPositionMode(2);
        ictrlL -> setPositionMode(3);
        goToRest();

    printMessage(0,"Closing ports..\n");
        closePort(skinPort);
        printMessage(1,"skin port successfully closed!\n");
        closePort(outPort);
        printMessage(1,"output port successfully closed!\n");

    printMessage(0,"Closing controllers..\n");
        ddR.close();
        ddL.close();
        Vector ang(3,0.0);
        // igaze -> lookAtAbsAngles(ang);
        // igaze -> restoreContext(contextGaze);
        // igaze -> stopControl();
        ddG.close();

    printMessage(0,"Closing solver..\n");
        delete slv; slv = NULL;
        delete gue; gue = NULL;
        delete sol; sol = NULL;

        delete testLimb; testLimb = NULL;
}

// empty line to make gcc happy
