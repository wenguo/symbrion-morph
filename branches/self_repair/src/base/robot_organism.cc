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
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

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

            InitRobotPoseInOrganism();

            //force to disconnect any unwanted ethernet connections
            std::vector<IPC::Connection*> *connections = master_IPC.Connections();
            for(int i=0;i<connections->size();i++)
            {
                uint32_t ip = (*connections)[i]->addr.sin_addr.s_addr;
                std::map<uint32_t, robot_pose>::iterator it;
                it=robot_pose_in_organism.find(ip);
                if(it!= robot_pose_in_organism.end())
                    printf("organism ip list: %d\n", ip>>24 & 0xFF);
                else 
                {
                    (*connections)[i]->Disconnect();
                   // printf("\tnot in organism ip list: %d\n", ip>>24 & 0xFF);
                }
            }


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
        else if((mytree.isAllIPSet()  && mytree.Size() !=0) && !IP_collection_done)
        //else if(msg_organism_seq_received && (mytree.isAllIPSet()  && mytree.Size() !=0) && !IP_collection_done)
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
            IPCSendMessage(neighbours_IP[parent_side].i32, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1);

        }
        else if(organism_formed)
        {
            msg_organism_seq_received  = false;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            /*
            msg_lowering_received = false;
            msg_raising_stop_received = false;
            msg_climbing_start_received =false;
            msg_climbing_stop_received =false;
            */
            //it is time to stop master_IPC, if I am used to be a seed
            master_IPC.Stop();

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
        if(raising_count < (uint32_t)raising_delay)
        {
            //waiting;
            //if(broken_eth_connections > 0)
            //    IPC_health = false;

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



                IPC_health = true;
                
             }

            //check if all robot 
            if(raising_count % 10 == 9)
            {
                std::map<uint32_t, robot_pose>::iterator it;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    //check if lost received some messages
                    if(commander_acks[it->first] < 1) //no response at all?
                    {
                        IPC_health = false;
                        printf("%d : WARNING! ip: %s acks %d\n", timestamp, IPToString(it->first), commander_acks[it->first] );
                    }
                    //reset the count
                    commander_acks[it->first] = 0;
                }
            }

        }
        else
        {
           // printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
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

    
#ifdef WITH_MOTION
    MoveHingeMotor(hinge_command);
#endif
    
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
        for(uint32_t i=0;i<og_reflective_sensors.left.size();i++)
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

        //only moves when user_input ==1
        if(user_input != 1)
            memset(cmd_speed, 0, sizeof(cmd_speed));

        if(user_input <= -2)
        {
            if(!msg_lowering_received)
            {
                printf("%d: send lowering start\n", timestamp);
                IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                msg_lowering_received = true;

                IPC_health = true;
                lowering_count =0;
                macrolocomotion_count = 0;
                hinge_motor_operating_count = 0;
            }
            user_input = 0;

        }
        else if(user_input >= 5)
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

        //printf("macrolocomotion speed: %d %d %d %d\t user_input:%d\n", cmd_speed[0], cmd_speed[1], cmd_speed[2], direction, user_input);


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
#ifdef WIP
        if(macrolocomotion_count > 50)
        {
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;

      //      if(!msg_climbing_start_received) //this will prevent the message being sent twice 
      //      {
      //          IPCSendMessage(IPC_MSG_CLIMBING_START, NULL, 0);
      //          msg_climbing_start_received = true; //a dirty fix to prevent message being sent twice as ethernet delay
      //      }
            if(!msg_lowering_received)
            {
                printf("%d: send lowering start\n", timestamp);
                IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                msg_lowering_received = true;
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
    if(type != ROBOT_AW &&timestamp - timestamp_user_input_received > 10)
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

#ifndef WITH_MOTION
    speed[0]=0;
    speed[1]=0;
    speed[2]=0;
#endif


}

void Robot::Climbing()
{
    climbing_count++;

    // Leds symbolise the raising process
    bool flash_leds = false;

    if(seed)
    {
        direction = FORWARD;

        if((uint32_t)current_action_sequence_index < organism_actions.size())
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
                if((uint32_t)current_action_sequence_index < organism_actions.size())
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
                for(uint32_t i=0;i<as_ptr->robots_in_action.size();i++)
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
                            printf("%d: stop! no need to move forward as AW detects the edge of the stairs\n", timestamp);
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
#ifdef WITH_MOTION
    MoveHingeMotor(hinge_command);
#endif

    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

    if(timestamp - timestamp_locomotion_motors_cmd_received > 3)
        memset(locomotion_command, 0, sizeof(locomotion_command));

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

void Robot::Lowering()
{
    lowering_count++;
    
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;


    // Leds symbolise the raising process
    bool flash_leds = false;

    if(seed)
    {
        uint32_t lowering_delay = 10;
        if(lowering_count < lowering_delay)
        {
            //waiting;
            //if(broken_eth_connections > 0)
            //    IPC_health = false;

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

#ifdef WIP
                if(!msg_reshaping_start_received)
                {
                    //prepare the buffer for new shaping + new seed
                    OrganismSequence::OrganismSequence &seq = para.og_seq_list[1];
                    int size = seq.Size() + 3;
                    uint8_t buf[size];
                    buf[0] = para.debug.para[9];
                    buf[1] = COMMANDER_PORT;
                    buf[2] = seq.Size();
                    for(unsigned int i=0; i < buf[2];i++)
                        buf[i+3] =seq.Encoded_Seq()[i].data;

                    IPCSendMessage(IPC_MSG_RESHAPING_START, buf, sizeof(buf));
                    printf("%d: send reshaping info to %d\n", timestamp, buf[0]);
                    seed = false;
                }
#else
                if(!msg_raising_start_received) //this will prevent the message being sent twice 
                {
                    IPCSendMessage(IPC_MSG_RAISING_START, NULL, 0);
                    msg_raising_start_received = true;
                }

                //if(!msg_disassembly_received) //this will prevent the message being sent twice 
               // {
               //     IPCSendMessage(MSG_TYPE_DISASSEMBLY, NULL, 0);
               //     msg_disassembly_received = true;
               // }
#endif           
                
                lowering_count = 0;
            }
            //check if all robot 
            if(raising_count % 10 == 9)
            {
                std::map<uint32_t, robot_pose>::iterator it;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    //check if lost received some messages
                    if(commander_acks[it->first] < 1) //no response at all?
                    {
                        IPC_health = false;
                        printf("%d : WARNING! ip: %s acks %d\n", timestamp, IPToString(it->first), commander_acks[it->first] );
                    }
                    //reset the count
                    commander_acks[it->first] = 0;
                }
            }

        }
        else
        {
            //printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
            flash_leds = true;
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
            msg_reshaping_start_received = false;
            reshaping_waiting_for_undock = 0xF; // all wait
            reshaping_processed = 0;
            reshaping_count = 0;

            organism_formed = false;
            IP_collection_done = false;

            commander_acks.clear();
            
            current_state = RESHAPING;
            last_state = LOWERING;
        }
    }

#ifdef WITH_MOTION
    MoveHingeMotor(hinge_command);
#endif
    
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

//Stage 0 -- outside of reshaping, set the new seed and new target tree, clean up all other's tree in old organism
//           stop all commander IPCs
//           the new seed start the master_IPC 
//Stage 1 -- all robots reconnect to tha new master_IPC, the older seed stop the master_IPC
void Robot::Reshaping()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    reshaping_count++;

    //delay a while to let socket closed
    //and reconnected
    if(reshaping_count <10)
        return;
    else if(reshaping_count < 20)
    {
        if(reshaping_count == 12)
        {
            //reconnect to the new seed
            commander_IPC.Start(commander_IP.i32, commander_port, false);
            printf("%d start IPC %s to %s:%d\n", timestamp, commander_IPC.Name(), IPToString(commander_IP), commander_port);

        }
        return;
    }



    //send new branches to connected neighbours
    //just do this once
    if(msg_organism_seq_received)
    {
        msg_organism_seq_received = false;

        // Prepare branch sequences
        rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
        if(ret.status >= RT_ERROR)
        {
            std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        // disable all LEDs
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0x0);

        std::vector<OrganismSequence>::iterator it = mybranches.begin();
        while(it !=mybranches.end())
        {
            bool erase_required = false;
            uint8_t branch_side = it->getSymbol(0).side1;
            if(docked[branch_side])
            {
                //if the older neighbour matches with the new require
                if(it->getSymbol(0) != OrganismSequence::Symbol(0) && it->getSymbol(0) == docked[branch_side])
                {
                    docking_done[branch_side] = true;
                    
                    //prepare the buffer for new shaping + new seed
                    OrganismSequence::OrganismSequence &seq = *it;
                    int size = seq.Size() + 3;
                    uint8_t buf[size];
                    buf[0] = (commander_IP.i32 >> 24) & 0xFF;
                    buf[1] = commander_port - COMMANDER_PORT_BASE;
                    buf[2] = seq.Size();
                    for(unsigned int i=0; i < buf[2];i++)
                        buf[i+3] =seq.Encoded_Seq()[i].data;

                    if(neighbours_IP[branch_side].i32 ==0)
                        printf("%d: ERROR!!!!!!!!!!, my neighbour %d should have an valid IP\n", timestamp, branch_side);
                    else
                        IPCSendMessage(neighbours_IP[branch_side].i32,IPC_MSG_ORGANISM_SEQ, buf, sizeof(buf));
                    printf("%d send organism_seq to %d(%s))\n", timestamp, branch_side, IPToString(neighbours_IP[branch_side]));
                    std::cout<<"seq: "<<*it<<std::endl;
                    erase_required = true;

                    reshaping_processed |= 1<<branch_side;

                    //set IPs in mytree
                    std::vector<uint8_t> root_IPs;
                    root_IPs.push_back(uint8_t((my_IP.i32 >>24) & 0xFF));
                    root_IPs.push_back(uint8_t((neighbours_IP[branch_side].i32>>24) & 0xFF));
                    mytree.setBranchRootIPs(robot_side(branch_side),root_IPs);
                }

            }

            if(erase_required)
                it = mybranches.erase(it);
            else
                ++it;

        }
    }

    //send reshaping done message to grigger unwanted robot to disassemble
    if(reshaping_count == 50 && seed)
    {
        printf("%d: send reshaping_done\n", timestamp);
        IPCSendMessage(IPC_MSG_RESHAPING_DONE, NULL, 0);
    }

    //reshpaing done -- notified from the seeds
    //do it once
    if(msg_reshaping_done_received)
    {
        printf("%d: received reshaping_done\n", timestamp);
        msg_reshaping_done_received = false;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i] && ((reshaping_processed & (1<<i)) ==0)) //docked but not processed
            {
                reshaping_waiting_for_undock |= 1<<i;
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
                reshaping_waiting_for_undock &= ~(1<<i);

            printf("docked: %#x reshaping_processed: %#x\n", docked[i], reshaping_processed);
        }
    }


    //waiting for undock
    for(int i=0;i<NUM_DOCKS;i++)
    {
        //received unlocked message
        if( (msg_unlocked_received & 1<<i) || ethernet_status_hist.Sum(i) < 1 )
        {
            msg_unlocked_received &= ~(1<<i);
            docked[i]=0;
            reshaping_waiting_for_undock &= ~(1<<i);
            
          //  EnablePowerSharing(i, false);//this may be sent multiple times
        }
        
        //motor fully opened
        if((reshaping_waiting_for_undock & (1<<i)) && unlocking_required[i] && locking_motors_status[i]==OPENED)
        {
            Robot::SendIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
            docked[i]=0;
            neighbours_IP[i]=0;
            unlocking_required[i] = false;

            reshaping_waiting_for_undock &= ~(1<<i);
        }
    }

    if(reshaping_waiting_for_undock == 0)
    {
        uint8_t recruiting_required = 0;
        std::vector<OrganismSequence>::iterator it = mybranches.begin();
        while(it !=mybranches.end())
        {
            uint8_t branch_side = it->getSymbol(0).side1;

            recruitment_count[branch_side] = 0;
            recruitment_signal_interval_count[branch_side] = DEFAULT_RECRUITMENT_COUNT;
            recruitment_stage[branch_side] = STAGE0;
            recruiting_required++;

            it++;
        }

        if(reshaping_processed == 0)
        {
            reshaping_waiting_for_undock = 0xF;
            reshaping_count = 0;
            msg_unlocked_received = 0;


            current_state = DISASSEMBLY;
            last_state = RESHAPING;
        }
        else if(recruiting_required)
        {
            msg_docking_signal_req_received = 0;
            msg_unlocked_received = 0;
            reshaping_waiting_for_undock = 0xF;
            reshaping_count = 0;
            reshaping_processed = 0;

            current_state = RECRUITMENT;
            last_state = RESHAPING;
        }
        else
        {
            reshaping_waiting_for_undock = 0xF;
            reshaping_count = 0;
            reshaping_processed = 0;

            msg_organism_seq_received = true; //to enable IP collection message
            msg_unlocked_received = 0;

            current_state = INORGANISM;
            last_state = RESHAPING;
        }

    }

    //flashing RGB leds
    static int index = 0;
    index = (timestamp / 2) % 6;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        switch (index)
        {
            case 0:
                SetRGBLED(i, BLUE, BLUE, BLUE, BLUE);
                break;
            case 1:
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

void Robot::Disassembly()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if(MessageWaitingAck(MSG_TYPE_PROPAGATED))
        return;

    disassembly_count++;

    if(disassembly_count == 1)
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
        disassembly_count = 0;
        undocking_count = 0;
        msg_unlocked_received = 0;

        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

        current_state = UNDOCKING;
        last_state = DISASSEMBLY;
    }

}

void Robot::Undocking()
{
    undocking_count++;

    //flashing led
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(timestamp % 4 ==0)
            SetRGBLED(i, 0,0,0,0);
        else
            SetRGBLED(i, RED,RED,RED,RED);
    }

    if( undocking_count > 100 )
    {
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;

        // Turn off LEDs
        for(int i=0;i<NUM_DOCKS;i++)
            SetRGBLED(i, 0,0,0,0);

        if( last_state ==  FAILED )
            current_state = RESTING;
        else
            current_state = FORAGING;

        last_state = UNDOCKING;

        RemoveFromAllQueues(IR_MSG_TYPE_UNLOCKED);
        ResetAssembly(); // reset variables used during assembly

        for(int i=0; i<SIDE_COUNT; i++)
            SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);
    }
    else
        Avoidance();
}

void Robot::Avoidance()
{

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    //for demo 
    static bool triggered = false;
    if(ambient_hist[0].Avg() > 1000 || ambient_hist[1].Avg() > 1000)
        triggered = true;
    else if(ambient_hist[4].Avg() > 1000 || ambient_hist[5].Avg() > 1000)
        triggered = false;
    if(!triggered)
        return;

    //front and rear are bumped 
    if((bumped & (1<<0 | 1<<1)) !=0 && (bumped & (1<<4 | 1<< 5))!=0)
    {
        speed[0] = 0;
        speed[1] = 0;
        direction = FORWARD;
    }
    else
    {
        speed[0] = 30;
        speed[1] = 30;
        //only front is bumped
        if((bumped & (1<<0 | 1<<1)) !=0) 
            direction  = BACKWARD;
        //only rear is bumped
        else if((bumped & (1<<4 | 1<< 5)) !=0) 
            direction  = FORWARD;
    }

    //move sideway if necessary
    if((bumped & (1<<2 | 1<<3)) !=0 && (bumped & (1<<6 | 1<<7)) !=0)
        speed[2] = 0;
    else if((bumped & (1<<2 | 1<<3)) !=0)
        speed[2] = -60;
    else if((bumped & (1<<6 | 1<<7)) !=0)
        speed[2] = 60;
    else
        speed[2] = 0;

    //last check ethernet port just in case
    if(ethernet_status_hist.Sum(0) >0 || ethernet_status_hist.Sum(2) >0 || 
            ethernet_status_hist.Sum(3) >0 || ethernet_status_hist.Sum(4) >0)
    {
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;
    }

}


void Robot::Seeding()
{
    mytree.Clear();

    if(!para.og_seq_list.empty())
    {
        mytree = target = para.og_seq_list[0];
        std::cout<<mytree<<std::endl;
    }
    else
        printf("Warning: empty organism sequence info\n");


    for(int i=0;i<SIDE_COUNT;i++)
    {
        recruitment_count[i] = 0;
        recruitment_signal_interval_count[i] = DEFAULT_RECRUITMENT_COUNT;
    }

    //start IPC thread, as a server
    master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);
    commander_IP = my_IP;
    commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;

    current_state = RECRUITMENT;
    last_state = SEEDING;

    msg_docking_signal_req_received = 0;
    seed = true;


    //prepare branches sequence
    rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }

    std::vector<OrganismSequence>::iterator it;

    //TODO: not to disable all
    //disable all ir leds first
    for(int i=0;i<NUM_DOCKS;i++)
        SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0x0); 
    //enable the recruiting side
    for(it = mybranches.begin() ; it != mybranches.end(); it++)
    {
        //check the first symbol that indicates the parent and child side of the connection
        uint8_t branch_side = it->getSymbol(0).side1;
        //enalbe docking signals
        SetIRLED(branch_side, IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1); 
        std::cout<<name<<" branch "<<*it<<std::endl;
    }
}

