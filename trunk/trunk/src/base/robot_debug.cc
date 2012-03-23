#include "robot.hh"

void Robot::Debugging()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    PrintAmbient();
    PrintReflective();
    PrintBeacon();
    PrintProximity();

    switch (para.testing_mode)
    {
        case 0:
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED1, 0);
            }
            break;
        case 1:
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDDOCKING, LED1, 0);
            }
            break;
        case 3: //for docking region detection threshold setting
            if(timestamp ==40)
            {
                SetIRLED(2, IRLEDDOCKING, LED1, IR_PULSE0|IR_PULSE1);
            }
            break;
        case 4: //for locking region detection threshold setting
            if(timestamp ==40)
            {
                SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0);
            }
            break;
        case 5: //for docking region detection, alignment --> docking
            {
                if(timestamp ==40)
                {
                    for(int i=0; i< NUM_DOCKS;i++)
                        SetIRLED(i, IRLEDOFF, LED1, IR_PULSE0 | IR_PULSE1);
                }
                Log();
            }
            break;
        case 6: //for locking region detection, docking-->locking
            {
                PrintReflective();
                PrintProximity();
                if(timestamp ==40)
                {
                    for(int i=0; i< NUM_DOCKS;i++)
                        SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
                }
                Log();
            }
            break;
        case 7:    
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);

                    if(timestamp ==50)
                        SetRGBLED(i, RED, RED, RED, RED);
                    if(timestamp ==60)
                        SetRGBLED(i, 0, 0, 0, 0);
                    //    SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);
                    if((msg_lockme_received & (1<<i)))
                    {
                        printf("start motor\n");
                        SetDockingMotor(i, CLOSE);
                        msg_lockme_received &=~(1<<i);
                        docking_done[i] = true;
                    }
                    else if((msg_unlockme_received & (1<<i)))
                    {
                        printf("start motor\n");
                        SetDockingMotor(i, OPEN);
                        msg_unlockme_received &=~(1<<i);
                        docking_done[i] = false;
                    }
                }
            }
            break;
        case 8:
            {
                if(CheckIRLEDStatus(2, 0)!=(0x4|IRLEDPROXIMITY))
                {
                    printf("set IR led ");
                    SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0);
                }
            }
            break;
        case 9:
            //      if(timestamp ==35)
            {
                leftspeed = para.speed_left_offset1;
                rightspeed = para.speed_right_offset1;
            }
            break;
        case 10:
            if(timestamp ==50)
            {
                printf("start motor\n");
                SetDockingMotor(2, CLOSE);
            }
            else if(timestamp == 100)
            {
                printf("start motor\n");
                SetDockingMotor(2, OPEN);
            }

                /* case 10:
                   {
                   if(timestamp == 50)
                   {
                   for(int i=0;i<NUM_DOCKS;i++)
                   SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);

                   SendAckRequiredMessage(2, MSG_LOCKED);
                   }
                   else if(timestamp == 55)
                   {
                   SendAckRequiredMessage(2, MSG_UNLOCKME);
                   }
                   else if(timestamp ==70)
                   {
                   mytree = new TreeSequence(og_cfg->GetOrganismSeq(g_organism_id)->seq.c_str());
                   seed = true;
                   tag = -1;

                //prepare branches sequence
                mytree->fillBranches();

                SendBranchTree(2);
                }
                else if(timestamp==150)
                {
                docked[1]=true;
                docked[2]=true;
                InitPropagateMessage(MSG_ORGANISM_FORMED);

                }
                }
                break;*/
        default:
                break;
    }
}
