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

	// Add one if the seed matches
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

		last_state = INORGANISM;
		current_state = FAILED;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, RED, RED, RED, RED);

		printf("%d Module failed!\n",timestamp);
		ret = true;

	}
	//else if( msg_failed_received )
	else if( para.debug.para[1] > 0 )
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

				last_state = INORGANISM;
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

		last_state = INORGANISM;
		current_state = REPAIR;
		repair_start = timestamp;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, YELLOW, YELLOW, YELLOW, YELLOW);

		msg_subog_seq_expected = 1<<wait_side;
		ret = true;

	}
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
			if( !MessageWaitingAck(IR_MSG_TYPE_SUB_OG_STRING, wait_side) )
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
		if( timestamp < move_start+move_delay )
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
			if( !MessageWaitingAck(IR_MSG_TYPE_SCORE_STRING, wait_side) )
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
			repair_stage = STAGE0;
			current_state = BROADCASTSCORE;
			last_state = REPAIR;
			printf("%d Score determined (%d), entering BROADCASTSCORE\n",timestamp,best_score);

			for( int i=0; i<NUM_DOCKS; i++ )
				SetRGBLED(i,MAGENTA,MAGENTA,MAGENTA,MAGENTA);

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
			if( !MessageWaitingAck(IR_MSG_TYPE_SUB_OG_STRING, wait_side) )
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
			if( timestamp > repair_start+repair_delay )
			{
				SendSubOGStr( parent_side, subog_str );

				wait_side = 0;
				own_score = -1;
				repair_stage = STAGE1;
				msg_score_seq_expected = 1 << parent_side;
				printf("%d Shape determined, entering STAGE1\n",timestamp);
			}
		}
	}
	// TODO: move away from failed module
	else if( repair_stage == STAGE1 )
	{

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
			if( !MessageWaitingAck(IR_MSG_TYPE_SCORE_STRING, wait_side) )
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
			if( timestamp > repair_start+repair_delay )
			{
				SendScoreStr( parent_side, subog, best_score );

				wait_side = 0;
				repair_stage = STAGE0;
				current_state = BROADCASTSCORE;
				last_state = REPAIR;
			}
		}
	}
}

void Robot::BroadcastScore()
{



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
