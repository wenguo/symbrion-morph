/*
 * robot_self_repair.cc
 *
 *  Created on: Apr 4, 2012
 *      Author: Lachlan Murray
 */

#include "robot.hh"

uint8_t Robot::calculateSubOGScore( OrganismSequence &seq1, OrganismSequence &seq2 )
{

	// get size of largest common sub-tree
	uint8_t score = OrganismSequence::maxCommonTreeSize( seq1, seq2 );

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
	//else if( msg_failed_received )
	else if( para.debug.para[1] >= 0 )
	{
		// for testing only //
		subog_id = 2;
		parent_side = para.debug.para[1];
		//////////////////////

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
			SendSubOGStr( wait_side, subog_str );

		msg_subog_seq_expected = 1<<wait_side;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i,GREEN,GREEN,GREEN,GREEN);

		// Turn side at which failed module detected red
		SetRGBLED(parent_side,RED,RED,RED,RED);

		current_state = LEADREPAIR;

		printf("%d Detected failed module, entering LEADREPAIR, sub-organism ID:%d\n",timestamp,subog_id);
		ret = true;
	}
	else if( msg_subog_seq_received )
	{
		msg_subog_seq_received = 0;
		repair_stage = STAGE0;

		// Find the first side at which another neighbour is docked (not parent)
		while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
			wait_side++;

		if( wait_side < SIDE_COUNT )
			SendSubOGStr( wait_side, subog_str );

		current_state = REPAIR;
		repair_start = timestamp;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, YELLOW, YELLOW, YELLOW, YELLOW);

		msg_subog_seq_expected = 1<<wait_side;
		ret = true;

	}
        return ret;
}

// Initial repair state of the robot nearest to the
// the failed/support module
void Robot::LeadRepair()
{
	// notify other modules to enter repair and determine shape of sub-organism
	if( repair_stage == STAGE0 )
	{
		// still waiting for some branches
		if( wait_side < SIDE_COUNT )
		{      
            // not waiting for acknowledgment from the previous message
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SUB_OG_STRING) )
			{
                if( msg_subog_seq_received & 1<<wait_side )
				{
					do // Find next neighbour (not including the failed module)
					{
						wait_side++;
					}
					while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side));

					if( wait_side < SIDE_COUNT )
						SendSubOGStr( wait_side, subog_str );

					msg_subog_seq_received = 0;
					msg_subog_seq_expected |= 1<<wait_side;
				}
			}
		}
		else
		{
			wait_side = 0;
			repair_stage = STAGE1;
			msg_subog_seq_expected = 0;
			printf("%d Shape determined, entering STAGE1\n",timestamp );
		    move_start = timestamp;
		}
	}
	// TODO: move away from failed module
	else if( repair_stage == STAGE1 )
	{
		// Flash LEDs whilst moving
		if( timestamp < move_start+move_duration )
		{
			int index;
			index = (timestamp / 2) % 4;
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
			for( int i=0; i<NUM_DOCKS; i++ )
				SetRGBLED(i,GREEN,GREEN,GREEN,GREEN);

			// convert sub-organism string to OrganismSequence
			subog.reBuild(subog_str+1,subog_str[0]);
			best_score = own_score = calculateSubOGScore( subog, target );

            std::cout << "OrganismSequence: " << subog << " score: " << (int) own_score << std::endl;

            wait_side = 0;
			// Find next neighbour (not including the failed module)
			while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
				wait_side++;

			if( wait_side < SIDE_COUNT )
				SendScoreStr( wait_side, subog, best_score );

			repair_stage = STAGE2;
			msg_score_seq_received = 0;
			msg_score_seq_expected = 1<<wait_side;
		}

	}
	// Determine sub-organism score
	else if( repair_stage == STAGE2 )
	{
		// still waiting for some branches
		if( wait_side < SIDE_COUNT )
		{
			// not waiting for acknowldgement from the previous message
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SCORE_STRING) )
			{
				if( msg_score_seq_received & 1<<wait_side )
				{
					// check if own_score no longer best
					if( own_score < best_score ) own_score = 0;

					do // Find next neighbour (not including the parent module)
					{
						wait_side++;
					}
					while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side));

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
			wait_side = 0;
			repair_stage = STAGE3;
			printf("%d Score determined (%d), entering STAGE3\n",timestamp,best_score);

			for( int i=0; i<NUM_DOCKS; i++ )
				SetRGBLED(i,MAGENTA,MAGENTA,MAGENTA,MAGENTA);

			best_id = subog_id;
			PropagateScore( best_id, best_score, parent_side );
                        broadcast_start = timestamp;
		}
	}
	// TODO: Broadcast and listen for sub-organism scores
	else if( repair_stage == STAGE3 )
	{
		if( timestamp < broadcast_start+broadcast_duration )
		{
			// Broadcast
			if( timestamp % broadcast_period == 0 )
			{
				BroadcastScore( parent_side, best_score, best_id );
			}
			// Listen
			else
			{
				// until a reshaping message is received
				//	- listen for sub-organism scores
				if( !(msg_reshaping_received & 1<<parent_side) )
				{
					int new_score_side = -1;
					for( int i=0; i<SIDE_COUNT; i++ )
					{
						if( msg_score_received & 1<<i && new_id[i] != best_id )
						{
							msg_score_received &= ~(1<<i);

							if( new_score[i] > best_score ||
							  ( new_score[i] == best_score && new_id[i] < best_id) )
							{
								best_score = new_score[i];
								best_id = new_id[i];
								new_score_side = i;
						                printf("%f received better score: %d %d\n",timestamp, best_id, best_score);
                                                        }

							new_score[i] = 0;
							new_id[i] = SIDE_COUNT;
						}
					}

					// If best score has changed, propagate messages to every
					// side - bar that which the new best score came from
					if( new_score_side > 0 )
						PropagateScore( best_id, best_score, new_score_side );
				}
			}
		}
		else
		{

			// If this is the winning organism
			if( best_id == subog_id )
				printf("%d This is the winning organism\n", timestamp );
			else
				printf("%d This is NOT the winning organism\n", timestamp );

			current_state = RESHAPING;
			last_state = LEADREPAIR;
			printf("%d negotiation finished, entering RESHAPING\n",timestamp);
		}
	}
}

// TODO: Initial repair state of the robot nearest
// to the failed module - not currently implemented
void Robot::Support()
{



}

// Initial repair state of remaining modules
void Robot::Repair()
{
	// determine sub-organism shape
	if( repair_stage == STAGE0 )
	{
		// still waiting for some branches
		if( wait_side < SIDE_COUNT )
		{
			// not waiting for acknowldgement from the previous message
			if( !MessageWaitingAck(wait_side,IR_MSG_TYPE_SUB_OG_STRING) )
			{
				if( msg_subog_seq_received & 1<<wait_side )
				{
					do // Find next neighbour (not including the parent module)
					{
						wait_side++;
					}
					while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side));

					if( wait_side < SIDE_COUNT )
						SendSubOGStr( wait_side, subog_str );

					msg_subog_seq_received = 0;
					msg_subog_seq_expected |= 1<<wait_side;
				}
			}
		}
		else
		{
			// make sure enough time has passed before
			// sending message back to parent module
			if( timestamp > repair_start+repair_duration )
			{
				SendSubOGStr( parent_side, subog_str );

				wait_side = 0;
				own_score = 0; // was -1
				repair_stage = STAGE1;
				msg_score_seq_expected = 1 << parent_side;
				printf("%d Shape determined, entering STAGE1\n",timestamp);
			}
		}
	}
	// TODO: move away from failed module
	else if( repair_stage == STAGE1 )
	{

		// Flash LEDs to signify that robots are moving
		if( !(msg_score_seq_received & 1<<parent_side) )
		{
			int index;
			index = (timestamp / 2) % 4;
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
			for( int i=0; i<NUM_DOCKS; i++ )
				SetRGBLED(i,YELLOW,YELLOW,YELLOW,YELLOW);

			// convert sub-organism string to OrganismSequence
			subog.reBuild(subog_str+1,subog_str[0]);
            subog = OrganismSequence::getNextSeedSeq(subog);
			own_score = calculateSubOGScore( subog, target );

			own_score > best_score ? best_score = own_score : own_score = 0;

			wait_side = 0;
			// Find next neighbour (not including the parent module)
			while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
				wait_side++;

			if( wait_side < SIDE_COUNT )
				SendScoreStr( wait_side, subog, best_score );

			repair_stage = STAGE2;
			repair_start = timestamp;
			msg_score_seq_received = 0;
			msg_score_seq_expected |= 1<<wait_side;
		}

	}
	else if( repair_stage == STAGE2 )
	{
		// still waiting for some branches
		if( wait_side < SIDE_COUNT )
		{
			// not waiting for acknowldgement from the previous message
			if( !MessageWaitingAck(wait_side, IR_MSG_TYPE_SCORE_STRING) )
			{
				if( msg_score_seq_received & 1<<wait_side )
				{
					if( own_score < best_score ) own_score = 0;

					do // Find next neighbour (not including the parent module)
					{
						wait_side++;
					}
					while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side));

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

				wait_side = 0;
				repair_stage = STAGE3;

				printf("%d Entering STAGE3\n",timestamp);

				for( int i=0; i<NUM_DOCKS; i++ )
					SetRGBLED(i,WHITE,WHITE,WHITE,WHITE);
			}
		}
	}
	// Listen for sub-organism scores
	else if( repair_stage == STAGE3 )
	{
		// until a reshaping message is received
		//	- listen for sub-organism scores
		if( !(msg_reshaping_received & 1<<parent_side) )
		{
			int new_score_side = -1;
			for( int i=0; i<SIDE_COUNT; i++ )
			{
				if( msg_score_received & 1<<i && new_id[i] != best_id )
				{
					msg_score_received &= ~(1<<i);

					if( new_score[i] > best_score ||
					  ( new_score[i] == best_score && new_id[i] < best_id) )
					{
						best_score = new_score[i];
						best_id = new_id[i];
						new_score_side = i;
					        printf("%f received better score: %d %d\n",timestamp, best_id, best_score);
                                        }


					new_score[i] = 0;
					new_id[i] = SIDE_COUNT;
				}
			}

			// If best score has changed, propagate messages to every
			// side - bar that which the new best score came from
			if( new_score_side > 0 )
				PropagateScore( best_id, best_score, new_score_side );
		}
		else
		{
			// TODO: reset everything to how it was before
			current_state = RESHAPING;
			last_state = REPAIR;
			printf("%d negotiation finished, entering RESHAPING\n",timestamp);

		}
	}
}


void Robot::Failed()
{

    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    /*
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
					docked[i]=false;
					num_docked--;
				}
			}
		}

		//only one  or less
		if(num_docked ==0)
		{
			current_state = UNDOCKING;
			last_state = FAILED;
		}
	}
	*/
}
