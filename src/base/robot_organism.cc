#include "robot.hh"
void Robot::InOrganism()
{
    speed[0] = 0;
    speed[1] = 0;

    //seed robot monitoring total number of robots in the organism
    if(seed)
    {
        if( mytree.Edges() + 1 == (unsigned int)num_robots_inorganism)
            printf("%d total number of robots in organism is %d\n", timestamp, mytree.Edges() + 1);

        if(mytree.isAllIPSet() && !IP_collection_done)
        {
            organism_formed = true;
            IP_collection_done = true;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism (%d) formed !!\n", timestamp, target.Encoded_Seq().size());
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            //prepare organism_formed_messages
            /*uint8_t buf[target.Encoded_Seq().size() + 2];
            for(int i = 0; i< target.Encoded_Seq().size(); i++)
                buf[i] = target.Encoded_Seq()[i].data;
            buf[target.Encoded_Seq().size()] = (my_IP.i32>>24)& 0xFF; //IP
            buf[target.Encoded_Seq().size() + 1] = COMMANDER_PORT;//commander port
            PropagateIRMessage(MSG_TYPE_ORGANISM_FORMED, buf, target.Encoded_Seq().size() + 2);
            PropagateEthMessage(MSG_TYPE_ORGANISM_FORMED, buf, target.Encoded_Seq().size() + 2);*/
            /*uint8_t data[2];
            data[0] = (my_IP.i32>>24) & 0xFF;
            data[1] =COMMANDER_PORT;
            PropagateIRMessage(MSG_TYPE_ORGANISM_FORMED, data, 2);
            PropagateEthMessage(MSG_TYPE_ORGANISM_FORMED, data, 2);*/

            uint8_t buf[target.Encoded_Seq().size()];
            for(uint32_t i = 0; i< target.Encoded_Seq().size(); i++)
                buf[i] = target.Encoded_Seq()[i].data;
            IPCSendMessage(MSG_TYPE_ORGANISM_FORMED, buf, sizeof(buf));

            
            //init the client list and the acks
            pthread_mutex_lock(&IPC_data_mutex);
            for(unsigned int i=0;i<mytree.Encoded_Seq().size();i++)
            {
                if(mytree.Encoded_Seq()[i] != OrganismSequence::Symbol(0))
                    commander_acks[getFullIP(mytree.Encoded_Seq()[i].child_IP).i32] = 0;
            }
            pthread_mutex_unlock(&IPC_data_mutex);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %s\n", IPToString(my_IP));
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
                EnablePowerSharing(i, true);
            }
        }
    }
    //otherwise check if new info received
    else
    {
        if(msg_organism_seq_expected && msg_organism_seq_received)
        {
            msg_organism_seq_expected = false;

            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, 0,0,0,0);

            //check if  need to recruit new robots
            std::vector<OrganismSequence>::iterator it = mybranches.begin();
            bool recruiting_required = false;
            while(it!=mybranches.end())
            {
                //check the first symbol that indicates the parent and child side of the connection
                uint8_t branch_side = it->getSymbol(0).side1;
                std::cout<<name<<" branch "<<*it<<std::endl;

                recruitment_count[branch_side] = 0;
                recruitment_signal_interval_count[branch_side] = DEFAULT_RECRUITMENT_COUNT;

                ++it;
                recruiting_required = true;
            }

            if(recruiting_required)
            {
                current_state = RECRUITMENT;
                last_state = INORGANISM;
                msg_docking_signal_req_received = 0;
            }
        }
        else if(msg_organism_seq_received && (mytree.isAllIPSet()  && mytree.Size() !=0) && !IP_collection_done)
        {
            IP_collection_done = true;
            //send the IPs to its parent
            std::vector<uint8_t> IPs;
            mytree.getAllIPs(IPs);
            uint8_t data[IPs.size()+1];
            data[0]=IPs.size();
            for(uint32_t i=0;i<IPs.size();i++)
                data[i+1]=IPs[i];
            SendIRMessage(parent_side, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1, para.ir_msg_repeated_num);
            //SendEthMessage(parent_side, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1, true);
            IPCSendMessage(neighbours_IP[parent_side].i32, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1);

        }
        else if(organism_formed)
        {
            msg_organism_seq_received  = 0;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %s\n", IPToString(my_IP));
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
                EnablePowerSharing(i, true);
            }

        }
    }

}

void Robot::Raising()
{
    //wait until all propapagated messages are gone
    if(MessageWaitingAck(MSG_TYPE_PROPAGATED))
        return;

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    
    raising_count++;

    // Leds symbolise the raising process
    bool flash_leds = false;

    // Wait longer with larger structures
    int raising_delay = 10;//(mytree.Size()/2+1)*30;

    if(seed)
    {
        //wait for a while until the propagated messages are done within the organism
        if(raising_count < raising_delay)
        {
            //waiting;
            if(broken_eth_connections > 0)
                IPC_health = false;

            memset(hinge_command, 0, sizeof(hinge_command));
        }
        else if(IPC_health)
        {
            hinge_motor_operating_count++;

            if(hinge_motor_operating_count < (uint32_t)para.hinge_motor_lifting_time)
            {
                hinge_command[0] = para.hinge_motor_angle;
                hinge_command[1] = para.hinge_motor_speed;
                hinge_command[2] = hinge_motor_operating_count ;
                hinge_command[3] = 1; //this indicates the validation of command
                IPCSendMessage(IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)hinge_command, sizeof(hinge_command));
            }
            else
            {
                if(!msg_raising_stop_received) //this will prevent the message being sent twice 
                {
                    printf("%d: send raising stop\n", timestamp);
                    IPCSendMessage(IPC_MSG_RAISING_STOP,NULL, 0);
                    msg_raising_stop_received = true;
                    IPCSendMessage(IPC_MSG_RESET_POSE_REQ,NULL, 0);
                }


                InitRobotPoseInOrganism();

                IPC_health = true;
                
             }

            //check if all robot 
            if(raising_count % 5 == 4)
            {
                std::map<uint32_t, int>::iterator it;
                for(it = commander_acks.begin(); it != commander_acks.end(); it++)
                {
                    //check if lost received some messages
                    if(it->second < 1) //no response at all?
                    {
                        IPC_health = false;
                        printf("%d : ip: %s acks %d\n", timestamp, IPToString(it->first), it->second );
                    }
                    //reset the count
                    it->second = 0;
                }
            }

        }
        else
        {
            printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
            flash_leds = true;
            memset(hinge_command, 0, sizeof(hinge_command));
        }
    }


    if( msg_raising_stop_received )
    {
        printf("%d: recieved raising stop\n", timestamp);

        msg_raising_stop_received = false;
        current_state = MACROLOCOMOTION;
        last_state = RAISING;
        raising_count = 0;
        flash_leds = false;
        hinge_motor_operating_count=0;
        macrolocomotion_count = 0;

        memset(hinge_command, 0, sizeof(hinge_command));

        direction = FORWARD;


        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED2, IRPULSE0|IRPULSE1);

    }
    else if( msg_raising_start_received )
    {
        msg_raising_start_received = false;
        flash_leds = true;
    }

    
    MoveHingeMotor(hinge_command);
    
    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

    if(flash_leds)
    {
        //flashing RGB leds
        static int index = 0;
        index = (timestamp / 2) % 6;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            switch (index)
            {
                case 0:
                    SetRGBLED(i, RED, RED, 0, 0);
                    break;
                case 1:
                    SetRGBLED(i, 0, 0, 0, 0);
                    break;
                case 2:
                    SetRGBLED(i, 0, 0, RED, RED);
                    break;
                case 3: //
                case 4: // short delay to better symbolise raising
                case 5: //
                    SetRGBLED(i, 0, 0, 0, 0);
                    break;
                default:
                    break;
            }
        }
    }
}

void Robot::MacroLocomotion()
{
    // Stop moving
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    macrolocomotion_count++;

    if(seed)
    {
        //PrintOGIRSensor(IR_REFLECTIVE_DATA);
        //request IRSensors
        RequestOGIRSensors(IR_REFLECTIVE_DATA);

        int cmd_speed[3] = {0,0,0};

        //make a decision for the speed of organism

        uint8_t organism_bumped = 0;
        //check front and back side
        for(int i=0;i<2;i++)
        {
            if(og_reflective_sensors.front[i] > 2000)
                organism_bumped |= 1;
            if(og_reflective_sensors.back[i] > 2000)
                organism_bumped |= 1<<2;
        }

        if((organism_bumped & 0x5) == 0x5) //both front and back are bumped
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            direction = FORWARD;
        }
        else
        {
            cmd_speed[0] = 30;
            cmd_speed[1] = 30;

            if((organism_bumped & 0x5) == 0x1)//front bumped
                direction = BACKWARD;
            else if((organism_bumped & 0x5) == 0x4)//back bumped
                direction = FORWARD;
        }

        //check left and right side
        for(int i=0;i<og_reflective_sensors.left.size();i++)
        {
            if(og_reflective_sensors.left[i] > 2000)
                organism_bumped |= 1<<1;
            if(og_reflective_sensors.right[i] > 2000)
                organism_bumped |= 1<<3;
        }

        //left and right
        if((organism_bumped & 0xA) == 0xA) //both left and right are bumped
            cmd_speed[2] = 0;
        else if((organism_bumped & 0xA) == 0x2) //left bumped
            cmd_speed[2] = -60;
        else if((organism_bumped & 0xA) == 0x8) //right bumped
            cmd_speed[2] = 60;
        else
            cmd_speed[2] = 0;


        if(user_input <= 0)
        {
            memset(cmd_speed, 0, sizeof(cmd_speed));
            if(user_input <= -3)
            {
                if(!msg_lowering_received)
                {
                    IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                    msg_lowering_received = true;

                    IPC_health = true;
                    lowering_count =0;
                    macrolocomotion_count = 0;
                    hinge_motor_operating_count = 0;
                }
                user_input = 0;

            }
        }
        else if(user_input >= 3)
        {
            if(!msg_climbing_start_received) //this will prevent the message being sent twice 
            {
         //       IPCSendMessage(IPC_MSG_CLIMBING_START, NULL, 0);
         //       msg_climbing_start_received = true; //a dirty fix to prevent message being sent twice as ethernet delay

                IPC_health = true;
                climbing_count =0;
                macrolocomotion_count = 0;
                hinge_motor_operating_count = 0;
            }
            user_input = 0;
        }

        printf("macrolocomotion speed: %d %d %d %d\t user_input:%d\n", cmd_speed[0], cmd_speed[1], cmd_speed[2], direction, user_input);


        /*
        if(macrolocomotion_count < 50)
        {
            cmd_speed[0] = 30;
            cmd_speed[1] = 30;
            cmd_speed[2] = 0;
        }
        else if(macrolocomotion_count < 100)
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            cmd_speed[2] = 60;
        }
        else if(macrolocomotion_count < 150)
        {
            cmd_speed[0] = -30;
            cmd_speed[1] = -30;
            cmd_speed[2] = 0;
        }
        else if(macrolocomotion_count < 200)
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            cmd_speed[2] = -60;
        }
        else
        */
#if 0
        if(macrolocomotion_count > 1000)
        {
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;

            if(!msg_climbing_start_received) //this will prevent the message being sent twice 
            {
                IPCSendMessage(IPC_MSG_CLIMBING_START, NULL, 0);
                msg_climbing_start_received = true; //a dirty fix to prevent message being sent twice as ethernet delay
            }
            if(!msg_lowering_received)
            {
     //           IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
     //           msg_lowering_received = true;
            }

            IPC_health = true;
            climbing_count =0;
            macrolocomotion_count = 0;
            hinge_motor_operating_count = 0;
        }

#endif
        //set the speed of all AW robots in the organism
        std::map<uint32_t, robot_pose>::iterator it;
        for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
        {
            int motor_command[4];
            if(it->second.type == ROBOT_AW)
            {
                motor_command[0] = direction * it->second.direction;
                motor_command[1] = motor_command[0] >0 ? cmd_speed[0] : cmd_speed[1];
                motor_command[2] = motor_command[0] >0 ? cmd_speed[1] : cmd_speed[0];
                motor_command[3] = direction * cmd_speed[2];
                IPCSendMessage(it->first, IPC_MSG_LOCOMOTION_2D_REQ, (uint8_t*)motor_command, sizeof(motor_command));
            }
            else
            {
                motor_command[0] = direction * it->second.direction;
                motor_command[1] = 0;
                motor_command[2] = 0;
                motor_command[3] = 0;
            }
        }
    }
    
    if( msg_lowering_received )
    {       
        printf("%d: received lowing start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        msg_lowering_received = false;
        last_state = MACROLOCOMOTION;
        current_state = LOWERING;
        lowering_count = 0;
        macrolocomotion_count=0;
    }
    else if (msg_climbing_start_received)
    {
        printf("%d: received climbing start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        msg_climbing_start_received = false;
        last_state = MACROLOCOMOTION;
        current_state = CLIMBING;
        climbing_count = 0;
        macrolocomotion_count=0;
        hinge_motor_operating_count = 0;
    }

    if(locomotion_command[0] != 0)
        direction = locomotion_command[0];
    speed[0] = locomotion_command[1];
    speed[1] = locomotion_command[2];
    speed[2] = locomotion_command[3];

    //reset
    if(timestamp - timestamp_locomotion_motors_cmd_received > 3)
        memset(locomotion_command, 0, sizeof(locomotion_command));


    //check ambient light to catch up some user input

    if(type != ROBOT_AW &&timestamp - timestamp_user_input_received > 30)
    {
        if(ambient_hist[2].Avg() > 1000 || ambient_hist[3].Avg() > 1000) //left
        {
            uint8_t command = 2;
            IPCSendMessage(commander_IP.i32, IPC_MSG_OPAQUE, &command, 1);
            timestamp_user_input_received = timestamp;

        }
        else if(ambient_hist[6].Avg() > 1000 || ambient_hist[5].Avg() > 1000)//right
        {
            uint8_t command = 0;
            IPCSendMessage(commander_IP.i32, IPC_MSG_OPAQUE, &command, 1);
            timestamp_user_input_received = timestamp;
        }
    }


    //flashing RGB leds
    static int index = 0;
    index = (timestamp / 2) % 4;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        switch (index)
        {
            case 0:
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);
                break;
            case 1:
                SetRGBLED(i, 0, 0, 0, 0);
                break;
            case 2:
                SetRGBLED(i, 0, 0, 0, 0);
                break;
            case 3:
                SetRGBLED(i, 0, 0, 0, 0);
                break;
            default:
                break;
        }
    }


}

void Robot::Climbing()
{
    climbing_count++;

    if(seed)
    {
        direction = FORWARD;

        if(current_action_sequence_index < organism_actions.size())
        {
            action_sequence * as_ptr = &organism_actions[current_action_sequence_index];
            as_ptr->counter++;

            //check if front_aw_ip is initialised fron the beginning
            if(current_action_sequence_index == 0 && front_aw_ip ==0)
            {
                std::map<uint32_t, robot_pose>::iterator it;
                bool flag = false;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    if(it->second.og_irsensor_index == 0)
                    {
                        flag = true;
                        front_aw_ip = it->first;
                        break;
                    }

                }
                if(!flag)
                    front_aw_ip = 0;
            }

            //end of life?
            if(as_ptr->counter >= as_ptr->duration)
            {
                current_action_sequence_index++;
                as_ptr->counter = 0; //reset the counter
                printf("%d the finished command is %s (%d)\n", timestamp, as_ptr->cmd_type == 0 ? "PUSH_DRAG":"LIFT_ONE", as_ptr->sequence_index);
              //  memset(hinge_command, 0, sizeof(hinge_command));
              //  memset(locomotion_command, 0, sizeof(locomotion_command));
                if(current_action_sequence_index < organism_actions.size())
                {
                    //if the latest action sequence is lifting, then next one should be push-drag
                    if(as_ptr->cmd_type == action_sequence::CMD_LIFT_ONE)
                    {
                        std::map<uint32_t, robot_pose>::iterator it;
                        bool flag = false;
                        for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                        {
                            if(it->second.og_irsensor_index == robot_pose_in_organism[front_aw_ip].og_irsensor_index + 1)
                            {
                                flag = true;
                                front_aw_ip = it->first;
                                break;
                            }
                        }

                        if(!flag)
                            front_aw_ip = 0;
                    }
                    printf("%d next command is %s (%d)\n", timestamp, organism_actions[current_action_sequence_index].cmd_type == 0 ? "PUSH_DRAG":"LIFT_ONE", organism_actions[current_action_sequence_index].sequence_index);
                }
                else
                    front_aw_ip = 0;

            }
            else
            {

                //request og_front_aux_reflective_sensor
                if(front_aw_ip != 0 && as_ptr->cmd_type == action_sequence::CMD_PUSH_DRAG)
                {
                    //printf("I am request aux_reflective_data from %s\n", IPToString(front_aw_ip));
                    RequestOGIRSensors(front_aw_ip, IR_AUX_REFLECTIVE_DATA);
                    //PrintOGIRSensor(IR_AUX_REFLECTIVE_DATA);
                }
                else
                {
                    memset(og_front_aux_reflective_sensors, 0 , sizeof(og_front_aux_reflective_sensors));
                }

                //set the command for each robots in the organism
                for(int i=0;i<as_ptr->robots_in_action.size();i++)
                {
                    uint32_t robot_ip=robot_in_organism_index_sorted[as_ptr->robots_in_action[i].index];
                    //printf("Send command [%d] to %s\n",as_ptr->cmd_type, IPToString(robot_ip));
                    int motor_command[4];


                    if(as_ptr->cmd_type == action_sequence::CMD_PUSH_DRAG)
                    {
                        if(og_front_aux_reflective_sensors[0] > 3000 ||
                           og_front_aux_reflective_sensors[1] > 3000 ||
                           og_front_aux_reflective_sensors[2] > 3000 ||
                           og_front_aux_reflective_sensors[3] >3000)
                        {
                            motor_command[0] = 0;
                            motor_command[1] = 0;
                            motor_command[2] = 0;
                            motor_command[3] = 0;
                            printf("%d: stop! no need to move forward as AW detects the edge of the stairs\n");
                        }
                        else
                        {
                            motor_command[0] = direction * robot_pose_in_organism[robot_ip].direction;
                            motor_command[1] = motor_command[0] >0 ? as_ptr->robots_in_action[i].cmd_data[0] : as_ptr->robots_in_action[i].cmd_data[1] ;
                            motor_command[2] = motor_command[0] >0 ? as_ptr->robots_in_action[i].cmd_data[1] : as_ptr->robots_in_action[i].cmd_data[0] ;
                            motor_command[3] = as_ptr->robots_in_action[i].cmd_data[2];
                        }
                        IPCSendMessage(robot_ip, IPC_MSG_LOCOMOTION_2D_REQ, (uint8_t*)motor_command, sizeof(motor_command));
                    }
                    else if(as_ptr->cmd_type == action_sequence::CMD_LIFT_ONE)
                    { 
                        if(as_ptr->counter + 1 >= as_ptr->duration)
                        {
                            //send a stop hinge command;
                            memset(motor_command, 0, sizeof(motor_command));
                        }
                        else
                        {
                            motor_command[0] = as_ptr->robots_in_action[i].cmd_data[0];
                            motor_command[1] = as_ptr->robots_in_action[i].cmd_data[1];
                            motor_command[2] = as_ptr->robots_in_action[i].cmd_data[2];
                            motor_command[3] = 1; //this indicates the validation of command
                        }

                        IPCSendMessage(robot_ip, IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)motor_command, sizeof(motor_command));
                    }

                }
            }
        }
        else
        {
            // Stop moving
            memset(hinge_command, 0, sizeof(hinge_command));
            memset(locomotion_command, 0, sizeof(locomotion_command));

            front_aw_ip = 0; //reset it to the right value 
            
            if(!msg_lowering_received)
            {
                printf("%d: send lowering start\n", timestamp);
                IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                msg_lowering_received = true;
            }
        }
    }


    if( msg_lowering_received )
    {
        printf("%d: received lowering start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        current_action_sequence_index =0;
        msg_lowering_received = false;
        last_state = CLIMBING;
        current_state = LOWERING;
        climbing_count =0;
        macrolocomotion_count=0;
        hinge_motor_operating_count = 0;
    }

    //2d locomotion will be called automatially
    if(locomotion_command[0] != 0)
        direction = locomotion_command[0];
    speed[0] = locomotion_command[1];
    speed[1] = locomotion_command[2];
    speed[2] = locomotion_command[3];

    //move hinge
    MoveHingeMotor(hinge_command);

    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

    if(timestamp - timestamp_locomotion_motors_cmd_received > 3)
        memset(locomotion_command, 0, sizeof(locomotion_command));
}

void Robot::Lowering()
{
    lowering_count++;
    
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if(seed)
    {
        int lowering_delay = 10;
        if(lowering_count < lowering_delay)
        {
            //waiting;
            if(broken_eth_connections > 0)
                IPC_health = false;

            memset(hinge_command, 0, sizeof(hinge_command));
        }
        else if(IPC_health)
        {
            hinge_motor_operating_count++;

            if(hinge_motor_operating_count < (uint32_t)para.hinge_motor_lowing_time)
            {
                hinge_command[0] = 7;
                hinge_command[1] = para.hinge_motor_speed;
                hinge_command[2] = hinge_motor_operating_count ;
                hinge_command[3] = 1; //this indicates the validation of command
                IPCSendMessage(IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)hinge_command, sizeof(hinge_command));
            }
            else
            {
                hinge_motor_operating_count = 0;

#if 1
                if(!msg_raising_start_received) //this will prevent the message being sent twice 
                {
                    IPCSendMessage(IPC_MSG_RAISING_START, NULL, 0);
                    msg_raising_start_received = true;
                }
#else
                if(!msg_disassembly_received) //this will prevent the message being sent twice 
                {
                    IPCSendMessage(MSG_TYPE_DISASSEMBLY, NULL, 0);
                    msg_disassembly_received = true;
                }
#endif           
                
                lowering_count = 0;
            }

            //check if all robot 
            if(lowering_count % 5 == 4)
            {
                std::map<uint32_t, int>::iterator it;
                for(it = commander_acks.begin(); it != commander_acks.end(); it++)
                {
                    //check if lost received some messages
                    if(it->second < 1) //no response?
                    {
                        IPC_health = false;
                        printf("%d : ip: %s acks %d\n", timestamp, IPToString(it->first), it->second );
                    }
                    //reset the count
                    it->second = 0;
                }
            }

        }
        else
        {
            printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
            memset(hinge_command, 0, sizeof(hinge_command));
        }
    }


    if(msg_disassembly_received)
    {
        current_state = DISASSEMBLY;
        last_state = LOWERING;
        lowering_count = 0;

        msg_disassembly_received = false;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if(docked[i])
                msg_unlocked_expected |= 1<<i;
        }

        memset(hinge_command, 0, sizeof(hinge_command));
    }
    else if(msg_raising_start_received)
    {
        msg_raising_start_received = false;
        macrolocomotion_count = 0;
        raising_count = 0;
        lowering_count = 0;
        current_state = RAISING;
        last_state = LOWERING;

        memset(hinge_command, 0, sizeof(hinge_command));
    }

    MoveHingeMotor(hinge_command);
    
    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

 }


