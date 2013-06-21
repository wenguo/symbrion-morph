/*
 * robot_self_repair.cc
 *
 *  Created on: Apr 4, 2012
 *      Author: Lachlan Murray
 */

#include "robot.hh"

// TODO:
//  1. Reduce code duplication

uint8_t Robot::calculateSubOrgScore( OrganismSequence &seq1, OrganismSequence &seq2 )
{

    // get size of largest common sub-tree
    uint8_t score = OrganismSequence::maxCommonTreeSize( seq1, seq2 );

    std::cout << "score: " << (int) score << std::endl;
    std::cout << "seq1: " << seq1 << std::endl;
    std::cout << "seq2: " << seq2 << std::endl;
    std::cout << "seq1 size: " << seq1.Encoded_Seq().size() << std::endl;
    std::cout << "seq2 size: " << seq2.Encoded_Seq().size() << std::endl;

    // Add one if the root matches
    if( seq1.Encoded_Seq().size() > 0 && seq2.Encoded_Seq().size() > 0 )
    {
        if( seq1.Encoded_Seq().front().type1 == seq2.Encoded_Seq().front().type1 )
            score++;
    }

    return score;

}

void Robot::CheckForFailures()
{
    bool start_recovery = false;

    if( module_failed )
    {
        switch(current_state)
        {
            case LOWERING:
            case RECRUITMENT:
            case RESHAPING:
            case DOCKING:
            case INORGANISM:
                {

                    if( current_state == LOWERING &&  lowering_count <= 30 ) break;
                    if( current_state == RECRUITMENT && recruitmentProgress() > STAGE3 ) break;
                    // TODO: check that IP addresses successfully exchanged
                    if( current_state == INORGANISM && (msg_organism_seq_expected && !msg_organism_seq_received) ) break;

                    // Send 'failed' message to all neighbours
                    for( int i=0; i<NUM_DOCKS; i++ )
                    {
                        if( docked[i] )
                            SendFailureMsg(i);
                    }

                    last_state = current_state;
                    current_state = FAILED;

                    for( int i=0; i<SIDE_COUNT; i++ )
                        SetRGBLED(i, RED, RED, RED, RED);

                    printf("%d Module failed, initiating self-repair\n",timestamp);
                    start_recovery = true;
                }
                break;
            case MACROLOCOMOTION:
            case RAISING:
                {
                    // Stop moving
                    speed[0] = 0;
                    speed[1] = 0;
                    speed[2] = 0;

                    // Propagate lowering messages
                    //PropagateIRMessage(MSG_TYPE_LOWERING);
                    IPCSendMessage(0, MSG_TYPE_LOWERING,NULL, 0);
                    msg_lowering_received = true;

                    last_state = current_state;
                    current_state = LOWERING;
                    lowering_count = 0;

                    printf("%d Module failed, initiating Lowering\n",timestamp);
                }
                break;
            default:
                break;
        }

    }
    else if( msg_failed_received )
    {
        switch(current_state)
        {
            case LOWERING:
            case RECRUITMENT:
            case RESHAPING:
            case INORGANISM:
                {

                    if( current_state == LOWERING &&  lowering_count <= 30 ) break;
                    if( current_state == RECRUITMENT && recruitmentProgress() > STAGE3 ) break;
                    // TODO: check that IP addresses successfully exchanged
                    if( current_state == INORGANISM && (msg_organism_seq_expected && !msg_organism_seq_received) ) break;

                    msg_failed_received = 0;

                    wait_side = 0;
                    repair_stage = STAGE0;
                    subog.Clear();
                    best_score = 0;
                    subog_str[0] = 0;

                    commander_IP = my_IP;
                    commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;


                    if( type == ROBOT_SCOUT && (heading == 1 || heading == 3) )
                    {
                        heading = 4;
                    }
                    else
                    {
                        // Check that neighbour will be able to move away properly
                        for( int i=0; i<SIDE_COUNT; i++ )
                        {
                            // If there is a robot docked
                            if( docked[i] )
                            {
                                // If the robot is a scout
                                if(  OrganismSequence::Symbol(docked[i]).type2 == ROBOT_SCOUT )
                                {
                                    int h = getNeighbourHeading(docked[i]);
                                    if(( h == 1 || h == 3 ) && heading == i )
                                    {
                                        heading = 4;
                                        std::cout << "I'm going to crash into my neighbour" << std::endl;
                                    }
                                }
                            }
                        }
                    }

                    //		wait_side = getNextNeighbour(0);
                    wait_side = getNextMobileNeighbour(0);

                    if( wait_side < SIDE_COUNT )
                    {
                        SendSubOrgStr( wait_side, subog_str );
                        msg_subog_seq_expected = 1<<wait_side;
                    }

                    for( int i=0; i<SIDE_COUNT; i++ )
                        SetRGBLED(i,GREEN,GREEN,GREEN,GREEN);

                    // Turn side at which failed module detected red
                    SetRGBLED(parent_side,RED,RED,RED,RED);

                    last_state = current_state;
                    current_state = LEADREPAIR;
                    msg_unlocked_expected |= 1<<parent_side;

                    printf("%d Detected failed module, entering LEADREPAIR, sub-organism ID:%d\n",timestamp,subog_id);
                    start_recovery = true;



                }
                break;
            default:
                break;
        }

    }
    else if( msg_subog_seq_received )
    {
        switch(current_state)
        {
            case LOWERING:
            case RECRUITMENT:
            case RESHAPING:
            case INORGANISM:
                {
                    if( current_state == LOWERING &&  lowering_count <= 30 ) break;
                    if( current_state == RECRUITMENT && recruitmentProgress() > STAGE3 ) break;
                    // TODO: check that IP addresses successfully exchanged
                    if( current_state == INORGANISM && (msg_organism_seq_expected && !msg_organism_seq_received) ) break;

                    // Check that neighbour will be able to move away properly
                    for( int i=0; i<SIDE_COUNT; i++ )
                    {
                        // If there is a robot docked
                        if( docked[i]  && (i != parent_side) ) // Ignore parent side
                        {
                            // If the robot is a scout
                            if(  OrganismSequence::Symbol(docked[i]).type2 == ROBOT_SCOUT )
                            {
                                int h = getNeighbourHeading(docked[i]);
                                if(( h == 1 || h == 3 ) && heading == i )
                                {
                                    heading = 4;
                                    std::cout << "I'm going to crash into my neighbour" << std::endl;
                                }
                            }
                        }
                    }


                    //msg_subog_seq_received = 0;

                    repair_stage = STAGE0;
                    best_score = 0;
                    own_score = 0;
                    subog.Clear();

                    wait_side = getNextMobileNeighbour(0);

                    if( wait_side < SIDE_COUNT )
                    {
                        SendSubOrgStr( wait_side, subog_str );
                        msg_subog_seq_expected = 1<<wait_side;
                    }

                    last_state = current_state;
                    current_state = REPAIR;
                    repair_start = timestamp;

                    for( int i=0; i<SIDE_COUNT; i++ )
                        SetRGBLED(i, YELLOW, YELLOW, YELLOW, YELLOW);

                    msg_subog_seq_expected = 1<<wait_side;

                    printf("%d Detected subog_seq, entering REPAIR %d\n",timestamp, wait_side);
                    start_recovery = true;
                }
                break;
            default:
                break;
        }
    }

    if( start_recovery )
    {
        lowering_count = 0;
        seed = false;
//        ResetAssembly(false);
        msg_score_received = 0;
        msg_stop_received = false;
        msg_retreat_received = false;
    }
}

// Reset variables when entering particular state
void Robot::changeState( fsm_state_t next_state )
{
    // Handle state specific variables
    switch(next_state)
    {
        case RESHAPING:
            msg_subog_seq_received = 0;
            msg_subog_seq_expected = 0;
            msg_failed_received = 0;
            repair_stage = STAGE0;
            best_score = 0;
            own_score = 0;
            subog.Clear();
            break;
        default:
            break;
    }

    last_state = current_state;
    current_state = next_state;

}

// Work out neighbours new heading based
//	upon own heading and docking info
uint8_t Robot::getNeighbourHeading( int8_t n )
{
    // If I can't move, it doesn't matter whether my neighbour can
    if( heading == 4 )
    {
        std::cout << timestamp << " neighbour can't move" << std::endl;
        return 4;
    }

    OrganismSequence::Symbol sym = OrganismSequence::Symbol(n);
    //	std::cout << timestamp << " docking info: " << sym << " s1:" << (int) sym.side1
    //								  	     << " s2:" << (int) sym.side2 << std::endl;

    int8_t o = (sym.side1+2) % 4;  // get opposite side
    int8_t d = (heading-o) % 4; 	// get the difference;
    if( d < 0 ) d += 4;			 	// handle negative result
    int8_t h = (sym.side2+d) % 4;	// calculate heading

    //std::cout << timestamp << " neighbour heading: " << (int) h << std::endl;

    return (uint8_t) h;


}

// Returns the recruiting state of the
//	side which has made the most progress
uint8_t Robot::recruitmentProgress()
{
    uint8_t stage = STAGE0;
    for( int i=0; i<SIDE_COUNT; i++ )
    {
        // TODO: remove once I have checked that IP addresses are successfully exchanged
        if( recruitment_stage[i] == STAGE5 ) continue;

        if( recruitment_stage[i] > stage )
            stage = recruitment_stage[i];
    }
    return stage;
}

void Robot::moveTowardsHeading( int h )
{
    // TODO: check that sidespeed set correctly

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if( h == 0 )
    {
        speed[0] = 30;
        speed[1] = 30;
    }
    else if( h == 1 )
    {
        speed[2] = 10;
    }
    else if( h == 2 )
    {
        speed[0] = -30;
        speed[1] = -30;
    }
    else
    {
        speed[2] = -10;
    }
}

uint8_t Robot::getNextNeighbour( int last )
{
    // Find the first side at which another module is
    //	docked - not including this modules parent
    while(last < SIDE_COUNT && (!docked[last] || last == parent_side))
        last++;

    return last;
}

uint8_t Robot::getNextMobileNeighbour( int last )
{

    bool neighbour_can_move = false;
    while( last < SIDE_COUNT && !neighbour_can_move )
    {
        if( !docked[last] || last == parent_side )
        {
            last++;
            continue;
        }

        // Check that this neighbour can move this way
        int h = getNeighbourHeading(docked[last]);
        if( OrganismSequence::Symbol(docked[last]).type2 == ROBOT_SCOUT )
        {
            if( h == 1 || h == 3 )
            {
                std::cout << timestamp << ": the neighbour on side " << last << " can't move this way " << std::endl;

                // Send failure message
                SendFailureMsg(last);
                pruning_required |= 1<<last;
                msg_unlocked_expected |= 1<<last;
                last = getNextNeighbour(++last);
            }
            else
            {
                neighbour_can_move = true;
            }
        }
        else
        {
            neighbour_can_move = true;
        }
    }

    return last;

}

// Check whether there a module docked using
// both infrared sensors and ethernet
bool Robot::isNeighbourConnected(int i)
{
    // threshold value;
    int a = 0;
    int p = 0;
    int r = 0;

    // If Ethernet enabled
    if( EthSwitch::isSwitchActive() )
    {
        //		return EthSwitch::switchIsPortConnected(i+1);

        // TODO: choose appropriate threshold values
        return ( EthSwitch::switchIsPortConnected(i+1) ||
                ( reflective_hist[2*i].Avg() > 100 && reflective_hist[2*i+1].Avg() > 100 ));
    }
    // If Ethernet not enabled - fall back to ir sensors
    else
    {
        // TODO: choose appropriate threshold values
        return ( reflective_hist[2*i].Avg() > 100 && reflective_hist[2*i+1].Avg() > 100 );

    }
}

/*
 * If a module is present on side 'channel' it
 * is sent the current sub-organism string.
 */
void Robot::SendSubOrgStr( int channel, uint8_t *seq )
{
    if( channel < SIDE_COUNT && docked[channel] )
    {
        uint8_t buf[MAX_IR_MESSAGE_SIZE-1];

        buf[0] = seq[0]+1;

        // TODO: if message too long, only send via Ethernet
        if( buf[0] > MAX_IR_MESSAGE_SIZE-5 )
        {
            printf("Warning: only %d of %d bytes will be sent (SendSubOGStr)\n", MAX_IR_MESSAGE_SIZE-2, (int) buf[0] );
            buf[0] = MAX_IR_MESSAGE_SIZE - 5;
        }

        // copy previous string
        memcpy(buf+1,seq+1,seq[0]);

        // if sending string back to parent
        if( channel == parent_side )
        {
            // add zeros
            buf[buf[0]] = 0;
        }
        else
        {
            buf[buf[0]] = 0;
            buf[buf[0]] |= type;	    // 0:1
            buf[buf[0]] |= channel<<2;  // 2:3
        }

        // Send the direction that the neighbour should move in
        buf[buf[0]+1] = getNeighbourHeading( docked[channel] );

        buf[buf[0] + 2] = (commander_IP.i32 >> 24 ) & 0xFF;
        buf[buf[0] + 3] = COMMANDER_PORT;

        SendIRMessage(channel, MSG_TYPE_SUB_OG_STRING, buf, buf[0]+4, para.ir_msg_repeated_num);
        //SendEthMessage(channel, MSG_TYPE_SUB_OG_STRING, buf, ((int)buf[0])+4, false);
        IPCSendMessage(neighbours_IP[channel].i32, MSG_TYPE_SUB_OG_STRING, buf, ((int)buf[0])+4);

        printf("%d Sending sub-og string, size: %d\n",timestamp,((int)buf[0])+4);
        PrintSubOGString(buf);
    }
}

void Robot::SendScoreStr( int channel, const OrganismSequence& seq, uint8_t score )
{

    if( channel < SIDE_COUNT && docked[channel] )
    {
        uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
        buf[0] = seq.Size();

        // TODO: if message too long, only send via Ethernet
        if( buf[0] > MAX_IR_MESSAGE_SIZE-2 )
        {
            printf("Warning: only %d of %d bytes will be sent (SendSubOGStr)\n", MAX_IR_MESSAGE_SIZE-2, (int) buf[0] );
            buf[0] = MAX_IR_MESSAGE_SIZE - 2;
        }

        for(unsigned int i=0; i < buf[0];i++)
            buf[i+1] =seq.Encoded_Seq()[i].data;

        buf[(int)(buf[0])+1] = score;

        //        std::cout << "sending score: " << (int) score << " and seq: " << seq
        //        		  << " size: " <<  ((int)buf[0])+2 << std::endl;
        SendIRMessage(channel, MSG_TYPE_SCORE_STRING, buf, buf[0]+2, para.ir_msg_repeated_num);
        //SendEthMessage(channel, MSG_TYPE_SCORE_STRING, buf, ((int)buf[0])+2, false);
        IPCSendMessage(neighbours_IP[channel].i32, MSG_TYPE_SCORE_STRING, buf, ((int)buf[0])+2);


    }
}

// Repair state of the robot nearest
// to the failed (or support) module
void Robot::LeadRepair()
{

    static uint8_t unlock_sent = 0; // Temporary solution
    static int lead_repair_count = 0;
    lead_repair_count++;
    if(lead_repair_count < 15)
    {
        if(lead_repair_count == 1)
        {
            //if(master_IPC.Running())
                master_IPC.Stop();
        }
        else if(lead_repair_count == 4)
        {
                master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);
        }
        else if(lead_repair_count == 7)
        {
           // if(commander_IPC.Running())
                commander_IPC.Stop();
        }
        else if(lead_repair_count == 10)
        {
            commander_IPC.Start(commander_IP.i32, commander_port, false);
        }

        return;
    }



    switch(repair_stage)
    {

        // Determine sub-organism shape
        case STAGE0:
            if( wait_side < SIDE_COUNT )
            {
                // not waiting for Ack from the previous message
                if( !MessageWaitingAck(wait_side, MSG_TYPE_SUB_OG_STRING) )
                {
                    printf("%d: wait_side %d msg_subog_seq_received %#x\n", timestamp, wait_side, msg_subog_seq_received);
                    if( msg_subog_seq_received & 1<<wait_side )
                    {
                        wait_side = getNextMobileNeighbour(++wait_side);

                        if( wait_side < SIDE_COUNT )
                        {
                            SendSubOrgStr( wait_side, subog_str );
                            msg_subog_seq_expected |= 1<<wait_side;
                        }

                        msg_subog_seq_received = 0;
                    }
                }
            }
            // Prune sub-structures
            else if( pruning_required )
            {
                printf("%d: wait_side %d pruning_required %#x parent_side: %d msg_unlocked_received: %#x\n", timestamp, wait_side, pruning_required, parent_side, msg_unlocked_received);
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    if(!MessageWaitingAck(i,MSG_TYPE_FAILED))
                    {
                        if( pruning_required & 1<<i )
                        {
                            if( (msg_unlocked_received & 1<<i) || !isNeighbourConnected(i) )
                            {
                                docked[i] = 0;
                                pruning_required &= ~(1<<i);

                                // No longer necessary to send UNLOCKED messages
                                RemoveFromQueue(i,IR_MSG_TYPE_UNLOCKED);
                            }
                            else if(unlocking_required[i])
                            {
                                SetDockingMotor(i, OPEN);
                                unlocking_required[i]=false;
                            }
                            else if( locking_motors_status[i]==OPENED && !(unlock_sent & 1<<i) )
                            {
                                SendIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                                unlock_sent |= 1<<i;
                            }
                        }
                    }
                }
            }
            // Un-dock from parent
            else if( !(unlock_sent & 1<<parent_side) )
            {
                printf("-- %d: wait_side %d unlock_sent %#x parent_side: %d unlocked_required: %#x\n", timestamp, wait_side, unlock_sent, parent_side, unlocking_required[parent_side]);
                if(unlocking_required[parent_side])
                {
                    SetDockingMotor(parent_side, OPEN);
                    unlocking_required[parent_side]=false;
                }
                else if( locking_motors_status[parent_side]==OPENED )
                {
                    BroadcastIRMessage(parent_side, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                    unlock_sent |= 1<<parent_side;
                    docked[parent_side]=0;
                }
            }
            else
            {
                // Reset variables
                unlock_sent = 0;
                docked[parent_side] = 0;
                msg_subog_seq_expected = 0;
                msg_unlocked_received = 0;
                msg_unlocked_expected = 0;

                //PropagateEthMessage(MSG_TYPE_RETREAT);
                PropagateIRMessage(MSG_TYPE_RETREAT);
                //IPCSendMessage(0, MSG_TYPE_RETREAT, NULL, 0);
                IPCPropagateMessage(MSG_TYPE_RETREAT);

                move_start = timestamp;

                repair_stage = STAGE1;

                std::cout << timestamp << ": Shape determined, entering STAGE1" << std::endl;
            }
            break;

            // Move away from failed module
        case STAGE1:

            if( timestamp < move_start+move_duration )
            {
                // Move away
                moveTowardsHeading(heading);

                // Flash LEDs whilst moving
                int index = (timestamp / 2) % 4;
                for( int i=0; i<NUM_DOCKS; i++ )
                {
                    switch(index)
                    {
                        case 0:
                        case 1:
                            SetRGBLED(i,BLUE,BLUE,BLUE,BLUE);
                            break;
                        case 2:
                        case 3:
                            SetRGBLED(i,0,0,0,0);
                            break;
                        default:
                            break;
                    }
                }
            }
            else
            {
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 0;

                //PropagateEthMessage(MSG_TYPE_STOP);
                PropagateIRMessage(MSG_TYPE_STOP);
                //IPCSendMessage(0, MSG_TYPE_STOP, NULL, 0);
                IPCPropagateMessage(MSG_TYPE_STOP);

                // Set LEDs
                for( int i=0; i<NUM_DOCKS; i++ )
                    SetRGBLED(i,GREEN,GREEN,GREEN,GREEN);

                repair_stage = STAGE2;
                wait_side = getNextNeighbour(0);

                // if there is only one module, create single module sequence
                if( subog_str[0] == 0 )
                    subog = OrganismSequence(type);
                else // else rebuild sequence from the subog_str
                    subog.reBuild(subog_str+1,subog_str[0]);

                best_score = own_score = calculateSubOrgScore( subog, target );
                std::cout << timestamp << ": OrganismSequence: " << subog << " score: " << (int) own_score << std::endl;

                if( wait_side < SIDE_COUNT )
                    SendScoreStr( wait_side, subog, best_score );

                msg_score_seq_received = 0;
                msg_score_seq_expected = 1<<wait_side;

                std::cout << timestamp << ": Sub organism removed, entering STAGE2" << std::endl;
            }
            break;

            // Determine sub-organism score
        case STAGE2:
            if( wait_side < SIDE_COUNT )
            {
                // not waiting for Ack from the previous message
                if( !MessageWaitingAck(wait_side, MSG_TYPE_SCORE_STRING) )
                {
                    if( msg_score_seq_received & 1<<wait_side )
                    {
                        // check if own_score no longer best
                        if( own_score < best_score ) own_score = 0;

                        wait_side = getNextNeighbour(++wait_side);

                        // rebuild next sequence
                        subog.reBuild(subog_str+1,subog_str[0]);
                        subog = OrganismSequence::getNextSeedSeq(subog);

                        if( wait_side < SIDE_COUNT )
                            SendScoreStr( wait_side, subog, best_score );

                        msg_score_seq_received = 0;
                        msg_score_seq_expected = 1<<wait_side;
                    }
                }
            }
            else
            {
                wait_side = getNextNeighbour(0);
                best_id = subog_id;
                broadcast_start = timestamp;

                // TODO: find out what the broadcast constant should be
                broadcast_period = 6 + (6*(target.Size()/2))-(6*(subog.Size()/2));


                for( int i=0; i<NUM_DOCKS; i++ )
                    SetRGBLED(i,MAGENTA,MAGENTA,MAGENTA,MAGENTA);

                repair_stage = STAGE3;
                std::cout << timestamp << ": Score determined (" << (int) best_score << "), entering STAGE3" << std::endl;
            }

            break;

            // Broadcast and listen for sub-organism scores
            // TODO: broadcast from other sides of the robot
        case STAGE3:

            if( timestamp < broadcast_start+broadcast_duration )
            {
                // Broadcast on each side in turn (if no one docked)
                int side = timestamp % RECRUITMENT_SIGNAL_INTERVAL;
                if( side < SIDE_COUNT )
                {
                    if( !docked[side] )
                    {
                        SetRGBLED(side,0,0,0,0);
                        BroadcastScore( side, best_score, best_id );
                    }
                }
                // Listen
                else
                {
                    SetRGBLED(parent_side,RED,RED,RED,RED);

                    int new_score_side = -1;
                    for( int i=0; i<SIDE_COUNT; i++ )
                    {
                        if( i !=parent_side )
                            SetRGBLED(i,MAGENTA,MAGENTA,MAGENTA,MAGENTA);

                        if( (msg_score_received & 1<<i) && (new_id[i] != best_id) )
                        {
                            msg_score_received &= ~(1<<i);

                            if( new_score[i] > best_score ||
                                    ( new_score[i] == best_score && new_id[i] < best_id) )
                            {
                                best_score = new_score[i];
                                best_id = new_id[i];
                                new_score_side = i;
                                printf("%d received better score: %d %d\n",timestamp, best_id, best_score);
                            }

                            new_score[i] = 0;
                            new_id[i] = SIDE_COUNT;
                        }
                    }

                    // If best score has changed, propagate messages to every
                    // side - bar that which the new best score came from
                    if( new_score_side > 0 )
                        PropagateSubOrgScore( best_id, best_score, new_score_side );
                }
            }
            else if( timestamp == broadcast_start+broadcast_duration )
            {
                // If  this is the winning organism
                if( best_id == subog_id )
                {
                    PropagateReshapeScore( best_score, parent_side );

                    if( best_score == own_score )
                    {
                        seed = true;
                        mytree = target;
                        std::cout<<"new target: "<<target<<std::endl;

                        //prepare the buffer for new shaping + new seed
                        OrganismSequence::OrganismSequence &seq = mytree;
                        int size = seq.Size() + 3;
                        uint8_t buf[size];
                        buf[0] = (my_IP.i32 >> 24 ) & 0xFF;
                        buf[1] = COMMANDER_PORT;
                        buf[2] = seq.Size();
                        for(unsigned int i=0; i < buf[2];i++)
                            buf[i+3] =seq.Encoded_Seq()[i].data;

                        IPCSendMessage(IPC_MSG_RESHAPING_START, buf, sizeof(buf));
                        printf("%d: send reshaping info to %d\n", timestamp, buf[0]);
                        seed = false;
                    }
                    else
                        msg_organism_seq_expected = true;

                    printf("%d This is the winning organism\n", timestamp );
                }
                else
                {
                    // add one to the best score so that no module will
                    // accidently believe itself to be the new seed.
                    PropagateReshapeScore( best_score+1, parent_side );
                    mytree.Clear();
                    seed = true;

                    printf("%d This is NOT the winning organism\n", timestamp );

                    //prepare the buffer for new shaping + new seed
                    OrganismSequence::OrganismSequence &seq = mytree;
                    int size = seq.Size() + 3;
                    uint8_t buf[size];
                    buf[0] = (my_IP.i32 >> 24 ) & 0xFF;
                    buf[1] = COMMANDER_PORT;
                    buf[2] = seq.Size();
                    for(unsigned int i=0; i < buf[2];i++)
                        buf[i+3] =seq.Encoded_Seq()[i].data;

                    IPCSendMessage(IPC_MSG_RESHAPING_START, buf, sizeof(buf));
                    printf("%d: send reshaping info to %d\n", timestamp, buf[0]);
                    seed = false;
                }
            }
            else if(msg_reshaping_start_received)
            {
                //start the new master_IPC if necessary
                if(reshaping_seed)
                {
                    //if it is not the older seed, then start a new
                    master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);

                    seed = reshaping_seed;
                    reshaping_seed = false;

                    msg_organism_seq_received = true;
                }

                //stop all commander IPCs
                if(commander_IPC.Running())
                {
                    commander_IPC.Stop();
                }
                else
                {
                    msg_subog_seq_received = 0;
                    msg_subog_seq_expected = 0;
                    msg_failed_received = 0;
                    repair_stage = STAGE0;
                    best_score = 0;
                    own_score = 0;
                    subog.Clear();

                    msg_reshaping_start_received = false;
                    reshaping_waiting_for_undock = 0xF; // all wait
                    reshaping_processed = 0;
                    reshaping_count = 0;
                    lead_repair_count = 0;

                    organism_formed = false;
                    IP_collection_done = false;


                    commander_acks.clear();

                    current_state = RESHAPING;
                    last_state = REPAIR;
                }
            }


            break;
    }

}

// Initial repair state of remaining modules
void Robot::Repair()
{
    static uint8_t unlock_sent = 0; // Temporary solution
    static int repair_count = 0;
    repair_count++;
    if(repair_count < 10)
    {
        if(repair_count == 1)
        {
            //stop the client 
            commander_IPC.Stop();
        }
        else if(repair_count == 4)
        {
            //if I used to be a seed, then stop the master
            master_IPC.Stop();
        }
        else if(repair_count == 7)
        {
            //start the new client to leader
            commander_IPC.Start(commander_IP.i32, commander_port, false);
        }
        return;
    }

    switch(repair_stage)
    {
        // Determine sub-organism score
        case STAGE0:

            if( wait_side < SIDE_COUNT )
            {
                // not waiting for Ack from the previous message
                if( !MessageWaitingAck(wait_side,MSG_TYPE_SUB_OG_STRING) )
                {
                    printf("%d: wait_side %d msg_subog_seq_received %#x\n", timestamp, wait_side, msg_subog_seq_received);
                    if( msg_subog_seq_received & 1<<wait_side )
                    {
                        wait_side = getNextMobileNeighbour(++wait_side);

                        if( wait_side < SIDE_COUNT )
                        {
                            SendSubOrgStr( wait_side, subog_str );
                            msg_subog_seq_expected |= 1<<wait_side;
                        }

                        msg_subog_seq_received = 0;
                    }
                }
            }
            // Prune sub-structures
            else if( pruning_required )
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    if(!MessageWaitingAck(i,MSG_TYPE_FAILED))
                    {
                        if( pruning_required & 1<<i )
                        {
                            if( (msg_unlocked_received & 1<<i) || !isNeighbourConnected(i) )
                            {
                                docked[i] = 0;
                                pruning_required &= ~(1<<i);
                            }
                            else if(unlocking_required[i])
                            {
                                SetDockingMotor(i, OPEN);
                                unlocking_required[i]=false;
                            }
                            else if( locking_motors_status[i]==OPENED && !(unlock_sent & 1<<i) )
                            {
                                SendIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                                unlock_sent |= 1<<i;
                            }
                        }
                    }
                }
            }
            else
            {
                // make sure enough time has passed before
                // sending message back to parent module
                if( timestamp > repair_start+repair_duration )
                {
                    // Reset variable
                    unlock_sent = 0;
                    msg_unlocked_received = 0;
                    msg_unlocked_expected = 0;

                    SendSubOrgStr( parent_side, subog_str );
                    repair_stage = STAGE1;
                    msg_score_seq_expected = 1 << parent_side;

                    std::cout << timestamp << ": Shape determined, entering STAGE1" << std::endl;
                }
            }
            break;

            // Move away from failed module
        case STAGE1:

            if( msg_retreat_received ) moveTowardsHeading(heading);

            if( msg_stop_received ) speed[0] = speed[1] = speed[2] = 0;

            if( msg_score_seq_received & 1<<parent_side )
            {
                speed[0] = speed[1] = speed[2] = 0;

                for( int i=0; i<NUM_DOCKS; i++ )
                    SetRGBLED(i,YELLOW,YELLOW,YELLOW,YELLOW);

                wait_side = getNextNeighbour(0);

                // convert sub-organism string to OrganismSequence
                subog.reBuild(subog_str+1,subog_str[0]);
                subog = OrganismSequence::getNextSeedSeq(subog);

                own_score = calculateSubOrgScore( subog, target );
                std::cout << timestamp << ": OrganismSequence: " << subog << " score: " << (int) own_score << std::endl;

                own_score > best_score ? best_score = own_score : own_score = 0;

                if( wait_side < SIDE_COUNT )
                    SendScoreStr( wait_side, subog, best_score );

                repair_stage = STAGE2;

                // reset flags
                msg_stop_received = false;
                msg_retreat_received = false;

                repair_start = timestamp;
                msg_score_seq_received = 0;
                msg_score_seq_expected |= 1<<wait_side;

                std::cout << timestamp << ": Sub organism removed, entering STAGE2" << std::endl;
            }
            else // Flash LEDs
            {
                int index = (timestamp / 2) % 4;
                for( int i=0; i<NUM_DOCKS; i++ )
                {
                    switch(index)
                    {
                        case 0:
                        case 1:
                            SetRGBLED(i,BLUE,BLUE,BLUE,BLUE);
                            break;
                        case 2:
                        case 3:
                            SetRGBLED(i,0,0,0,0);
                            break;
                        default:
                            break;
                    }
                }
            }

            break;

            // Determine sub-og score
        case STAGE2:
            // still waiting for some branches
            if( wait_side < SIDE_COUNT )
            {
                // not waiting for acknowldgement from the previous message
                if( !MessageWaitingAck(wait_side, MSG_TYPE_SCORE_STRING) )
                {
                    if(  msg_score_seq_received & 1<<wait_side )
                    {
                        if( own_score < best_score ) own_score = 0;

                        wait_side = getNextNeighbour(++wait_side);

                        if( wait_side < SIDE_COUNT )
                            SendScoreStr( wait_side, subog, best_score );

                        msg_score_seq_received = 0;
                        msg_score_seq_expected |= 1<<wait_side;
                    }
                }
            }
            else
            {
                // make sure enough time has passed before
                // sending message back to parent module
                if( timestamp > repair_start+repair_duration )
                {
                    SendScoreStr( parent_side, subog, best_score );

                    wait_side = getNextNeighbour(0);
                    repair_stage = STAGE3;
                    msg_reshaping_expected = 1<<parent_side;

                    std::cout << timestamp << ": Score determined, entering STAGE3" << std::endl;

                    for( int i=0; i<NUM_DOCKS; i++ )
                        SetRGBLED(i,WHITE,WHITE,WHITE,WHITE);
                }

            }
            break;

            // Listen for sub-organism scores
            // TODO: broadcast scores as well
        case STAGE3:
            if(msg_reshaping_start_received)
            {
                //start the new master_IPC if necessary
                if(reshaping_seed)
                {
                    //if it is not the older seed, then start a new
                    master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);

                    seed = reshaping_seed;
                    reshaping_seed = false;

                    msg_organism_seq_received = true;
                }

                //stop all commander IPCs
                if(commander_IPC.Running())
                {
                    commander_IPC.Stop();
                }
                else
                {
                    msg_subog_seq_received = 0;
                    msg_subog_seq_expected = 0;
                    msg_failed_received = 0;
                    repair_stage = STAGE0;
                    best_score = 0;
                    own_score = 0;
                    subog.Clear();

                    msg_reshaping_start_received = false;
                    reshaping_waiting_for_undock = 0xF; // all wait
                    reshaping_processed = 0;
                    reshaping_count = 0;
                    repair_count = 0;

                    organism_formed = false;
                    IP_collection_done = false;

                    commander_acks.clear();

                    current_state = RESHAPING;
                    last_state = REPAIR;
                }
            }
            // until a reshaping score message is received
            //	- listen for sub-organism scores TODO: this may be moved below
            else if((msg_reshaping_score_received & (1<<parent_side)) == 0 )
            {
                int new_score_side = -1;
                for( int i=0; i<SIDE_COUNT; i++ )
                {
                    if( (msg_score_received & 1<<i) && (new_id[i] != best_id) )
                    {
                        msg_score_received &= ~(1<<i);

                        if( new_score[i] > best_score ||
                                ( new_score[i] == best_score && new_id[i] < best_id) )
                        {
                            best_score = new_score[i];
                            best_id = new_id[i];
                            new_score_side = i;
                            printf("%d received better score: %d %d\n",timestamp, best_id, best_score);
                        }

                        new_score[i] = 0;
                        new_id[i] = SIDE_COUNT;
                    }
                }

                // If best score has changed, propagate messages to every
                // side - bar that which the new best score came from
                if( new_score_side > 0 )
                    PropagateSubOrgScore( best_id, best_score, new_score_side );

            }
            else if(msg_reshaping_score_received)
            {
                msg_reshaping_score_received = 0;

                if( best_score == own_score )
                {
                    seed = true;
                    mytree = target;
                    std::cout<<"new target: "<<target<<std::endl;
                    //num_robots_in_organism = 0;

                    //prepare the buffer for new shaping + new seed
                    OrganismSequence::OrganismSequence &seq = target;
                    int size = seq.Size() + 3;
                    uint8_t buf[size];
                    buf[0] = (my_IP.i32 >> 24 ) & 0xFF;
                    buf[1] = COMMANDER_PORT;
                    buf[2] = seq.Size();
                    for(unsigned int i=0; i < buf[2];i++)
                        buf[i+3] =seq.Encoded_Seq()[i].data;

                    IPCSendMessage(IPC_MSG_RESHAPING_START, buf, sizeof(buf));
                    printf("%d: send reshaping info to %d\n", timestamp, buf[0]);
                    seed = false;

                }
                else
                    msg_organism_seq_expected = true;

            }
            


            break;
    }
}

void Robot::Failed()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    static int failed_count=0;

    if(MessageWaitingAck(MSG_TYPE_FAILED))
        return;

    failed_count++;

    if(failed_count == 1)
    {
        //disconnect if there is any
        master_IPC.Stop();
        commander_IPC.Stop();

        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i]) //docked but not processed
            {
                disassembly_waiting_for_undock |= 1<<i;
                //open motor if its locking motor is closed
                if(unlocking_required[i])
                {
                    SetDockingMotor(i, OPEN);
                }
                else  
                    msg_unlocked_expected |= 1<<i; //waiting for undocked message

                //no need to send unlock me request, just wait for unlocked
                EnablePowerSharing(i, false);//this may be sent multiple times

            }
            else
                disassembly_waiting_for_undock &= ~(1<<i);

        }
    }

    //waiting for undock
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(disassembly_waiting_for_undock & (1<<i))
        {
            //received unlocked message
            if( (msg_unlocked_received & 1<<i) || ethernet_status_hist.Sum(i) < 1 )
            {
                msg_unlocked_received &= ~(1<<i);
                docked[i]=0;
                neighbours_IP[i]=0;
                disassembly_waiting_for_undock &= ~(1<<i);
            }

            //motor fully opened
            if(unlocking_required[i] && locking_motors_status[i]==OPENED)
            {
                Robot::SendIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                docked[i]=0;
                neighbours_IP[i]=0;
                unlocking_required[i] = false;

                disassembly_waiting_for_undock &= ~(1<<i);
            }
        }
    }

    if(disassembly_waiting_for_undock ==0)
    {
        disassembly_waiting_for_undock = 0xF;
        failed_count = 0;
        undocking_count = 0;
        msg_unlocked_received = 0;
        module_failed = false;

        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

        current_state = UNDOCKING;
        last_state = FAILED;
    }

}


#if 0
void Robot::Failed()
{

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    static uint8_t unlock_sent = 0;

    if(!MessageWaitingAck(MSG_TYPE_FAILED))
    {
        //check if need to unlocking docking faces which is connected to Activewheel
        int num_docked = 0;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i])
            {
                //std::cout << "robot docked on side " << i << std::endl;
                num_docked++;
                // AW
                if( type == ROBOT_AW )
                {
                    if( !(unlock_sent & 1<<i) )
                    {
                        BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                        unlock_sent |= 1<<i;
                    }
                    else if ((msg_unlocked_received & 1<<i) || !EthSwitch::switchIsPortConnected(i/2) )
                    {
                        docked[i]=0;
                        num_docked--;
                    }
                }
                // SCOUT or KIT
                else if(unlocking_required[i])
                {
                    SetDockingMotor(i, OPEN);
                    unlocking_required[i]=false;
                }
                else if(locking_motors_status[i]==OPENED)
                {
                    if( !(unlock_sent & 1<<i) )
                    {
                        BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                        unlock_sent |= 1<<i;
                    }
                    else if ((msg_unlocked_received & 1<<i) || !EthSwitch::switchIsPortConnected(i+1) )
                    {
                        docked[i]=0;
                        num_docked--;
                    }
                }
            }
        }

        //only one  or less
        if(num_docked ==0)
        {       
            unlock_sent = 0;
            undocking_count = 0;
            current_state = UNDOCKING;
            last_state = FAILED;
            module_failed = false;
        }
    }

}
#endif
// TODO: Initial repair state of the robot nearest
// to the failed module - not currently implemented
void Robot::Support()
{



}
