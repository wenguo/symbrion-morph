/*
 * robot_self_repair.cc
 *
 *  Created on: Apr 4, 2012
 *      Author: Lachlan Murray
 */

#include "robot.hh"

// TODO:
//  1. Reduce code duplication
//	2. Integrate ethernet

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

bool Robot::StartRepair()
{
	bool ret = false;
	if( module_failed )
	{
		// Send 'failed' message to all neighbours
		for( int i=0; i<NUM_DOCKS; i++ )
		{
			if( docked[i] )
				SendFailureMsg(i);
		}

		current_state = FAILED;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, RED, RED, RED, RED);

		printf("%d Module failed!\n",timestamp);
		ret = true;
	}
	else if( msg_failed_received )
	{
	//////// for testing only ////////
	//else if( para.debug.para[1] >= 0 )
	//{
	//	//subog_id = 2;
	//	subog_id = parent_side = para.debug.para[1];
        //    
	//////// for testing only ////////

		msg_failed_received = 0;

		wait_side = 0;
		repair_stage = STAGE0;
		subog.Clear();
		best_score = 0;
		subog_str[0] = 0;

		// Find the first side at which another neighbour is docked (not failed module)
		while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
			wait_side++;

		if( wait_side < SIDE_COUNT )
			SendSubOrgStr( wait_side, subog_str );

		msg_subog_seq_expected = 1<<wait_side;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i,GREEN,GREEN,GREEN,GREEN);

		// Turn side at which failed module detected red
		SetRGBLED(parent_side,RED,RED,RED,RED);

		current_state = LEADREPAIR;
		msg_unlocked_expected |= 1<<parent_side;

		printf("%d Detected failed module, entering LEADREPAIR, sub-organism ID:%d\n",timestamp,subog_id);
		ret = true;
	}
	else if( msg_subog_seq_received )
	{
		msg_subog_seq_received = 0;

		wait_side = 0;
		repair_stage = STAGE0;
		best_score = 0;

		// Find the first side at which another neighbour is docked (not parent)
		while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
			wait_side++;

		if( wait_side < SIDE_COUNT )
			SendSubOrgStr( wait_side, subog_str );

		current_state = REPAIR;
		repair_start = timestamp;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, YELLOW, YELLOW, YELLOW, YELLOW);

		msg_subog_seq_expected = 1<<wait_side;
        printf("%d Detected subog_seq, entering REPAIR\n",timestamp);
        ret = true;
	}
    return ret;
}

// Reset variables when entering particular state
void Robot::changeState( fsm_state_t next_state )
{
	// Handle state specific variables
	switch(next_state)
	{
	case RESHAPING:
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
uint8_t Robot::getNeighbourHeading( uint8_t n )
{
	OrganismSequence::Symbol sym = OrganismSequence::Symbol(n);
//	std::cout << id << " docking info: " << sym << " s1:" << (int) sym.side1
//								  	     << " s2:" << (int) sym.side2 << std::endl;

	uint8_t o = (sym.side1+2) % 4;  // get opposite side
	uint8_t d = (heading-o) % 4; 	// get the difference;
	if( d < 0 ) d += 4;			 	// handle negative result
	uint8_t h = (sym.side2+d) % 4;	// calculate heading

//	std::cout << id << " neighbour heading: " << (int) h << std::endl;

	return h;


}

void Robot::moveTowardsHeading( int h )
{
	// TODO: check that sidespeed set correctly

	leftspeed = 0;
	rightspeed = 0;
	sidespeed = 0;

	if( h == 0 )
	{
		leftspeed = 30;
		rightspeed = 30;
	}
	else if( h == 1 )
	{
		sidespeed = 10;
	}
	else if( h == 2 )
	{
		leftspeed = -30;
		rightspeed = -30;
	}
	else
	{
		sidespeed = -10;
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

// Repair state of the robot nearest
// to the failed (or support) module
void Robot::LeadRepair()
{
	switch(repair_stage)
	{

	// Determine sub-organism shape
	case STAGE0:
		if( wait_side < SIDE_COUNT )
		{
		    // not waiting for Ack from the previous message
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SUB_OG_STRING) )
			{
				if( msg_subog_seq_received & 1<<wait_side )
				{
					wait_side = getNextNeighbour(++wait_side);

					if( wait_side < SIDE_COUNT )
					{
						SendSubOrgStr( wait_side, subog_str );
						msg_subog_seq_expected |= 1<<wait_side;
					}

					msg_subog_seq_received = 0;
				}
			}
		}
		// TODO: disconnect from parent
		else if( msg_unlocked_received )
		{
			// TODO: Broadcast RETREAT msg using ethernet
			//BroadcastEthMessage(ETH_MSG_TYPE_RETREAT);
			PropagateIRMessage(IR_MSG_TYPE_RETREAT);

			repair_stage = STAGE1;
			docked[parent_side] = 0;
			msg_subog_seq_expected = 0;

			move_start = timestamp;

			std::cout << timestamp << ": Shape determined, entering STAGE1" << std::endl;
		}
		break;

	// Move away from failed module
	case STAGE1:

		if( timestamp < move_start+move_duration/2 )
		{
			// Short delay to allow retreat message to arrive
		}
		else if( timestamp < move_start+move_duration )
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
			// Stop moving
			leftspeed = 0;
		    rightspeed = 0;
		    sidespeed = 0;

			// TODO: Broadcast STOP msg using ethernet
			//BroadcastEthMessage(ETH_MSG_TYPE_STOP);
		    PropagateIRMessage(IR_MSG_TYPE_STOP);

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
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SCORE_STRING) )
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
			broadcast_period = 6 + (6*(target.Size()/2))-(6*(subog.Size()/2));

			// TODO: find out what the broadcast constant should be

			for( int i=0; i<NUM_DOCKS; i++ )
				SetRGBLED(i,MAGENTA,MAGENTA,MAGENTA,MAGENTA);

			repair_stage = STAGE3;
			std::cout << timestamp << ": Score determined (" << (int) best_score << "), entering STAGE3" << std::endl;
		}

		break;

	// Broadcast and listen for sub-organism scores
	case STAGE3:

		if( timestamp < broadcast_start+broadcast_duration )
		{
			// Broadcast
			if( timestamp % RECRUITMENT_SIGNAL_INTERVAL == 0 )
			{
				SetRGBLED(parent_side,0,0,0,0);
				// TODO: send a burst instead of a single message?
				BroadcastScore( parent_side, best_score, best_id );
			}
			// Listen
			else
			{
				SetRGBLED(parent_side,RED,RED,RED,RED);

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
		}
		else
		{
			// If  this is the winning organism
			if( best_id == subog_id )
			{
				PropagateReshapeScore( best_score, parent_side );
			
				if( best_score == own_score )
				{
					seed = true;
					mytree = target;
				}
				else // not the seed so expect to receive org seq
				{
					msg_organism_seq_expected = true;
				}

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
			}

			changeState(RESHAPING);
			printf("%d negotiation finished, entering RESHAPING\n",timestamp);
		}
		break;
	}
}

// Initial repair state of remaining modules
void Robot::Repair()
{

	switch(repair_stage)
	{
	// Determine sub-organism score
	case STAGE0:

		if( wait_side < SIDE_COUNT )
		{
			// not waiting for Ack from the previous message
			if( !MessageWaitingAck(wait_side,IR_MSG_TYPE_SUB_OG_STRING) )
			{
				if( msg_subog_seq_received & 1<<wait_side )
				{
	            	wait_side = getNextNeighbour(++wait_side);

	            	if( wait_side < SIDE_COUNT )
	            	{
	            		SendSubOrgStr( wait_side, subog_str );
						msg_subog_seq_expected |= 1<<wait_side;
	            	}

					msg_subog_seq_received = 0;
				}
			}
		}
		else
		{
			// make sure enough time has passed before
			// sending message back to parent module
			if( timestamp > repair_start+repair_duration )
			{
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

		if( msg_stop_received ) leftspeed = rightspeed = sidespeed = 0;

		if( msg_score_seq_received & 1<<parent_side )
		{
			leftspeed = rightspeed = sidespeed = 0;

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
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SCORE_STRING) )
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
	case STAGE3:

		// until a reshaping message is received
		//	- listen for sub-organism scores
		if( !(msg_reshaping_received & 1<<parent_side) )
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
		else
		{
			msg_reshaping_received = 0;

			if( best_score == own_score )
			{
				seed = true;
				mytree = target;
				//num_robots_in_organism = 0;
			}
			else
			{
				msg_organism_seq_expected = true;
			}

			changeState(RESHAPING);

		}
		break;
	}
}


void Robot::Failed()
{

    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    
	if(!MessageWaitingAck(IR_MSG_TYPE_FAILED))
	{
		//check if need to unlocking docking faces which is connected to Activewheel
		int num_docked = 0;
		for(int i=0;i<NUM_DOCKS;i++)
		{
			if(docked[i])
			{
				num_docked++;
				if(unlocking_required[i])
				{
					SetDockingMotor(i, OPEN);
					unlocking_required[i]=false;
				}
				//TODO: how about two KIT robots docked to each other
				else if(docking_motors_status[i]==OPENED)
				{
					BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, true);
					docked[i]=0;
					num_docked--;
				}
			}
		}

		//only one  or less
		if(num_docked ==0)
		{       
            undocking_count = 0;
			current_state = UNDOCKING;
			last_state = FAILED;
	        module_failed = false;
        }
	}
	
}

// TODO: Initial repair state of the robot nearest
// to the failed module - not currently implemented
void Robot::Support()
{



}
