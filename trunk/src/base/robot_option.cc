#include "robot.hh"

bool Robot::LoadParameters(const char * filename)
{
    if(!filename)
        return false;

    static bool loaded = false;

    if(loaded)
        return true;

    loaded = true;
    optionfile = new Morph::Worldfile();
    if(optionfile->Load(filename)==-1)
    {
        std::cout<<"can not find option.cfg, please check the path"<<std::endl;
        return false;
    }

    for( int entity = 1; entity < optionfile->GetEntityCount(); ++entity )
    {
        const char *typestr = (char*)optionfile->GetEntityType(entity);          

        // don't load window entries here
        if( strcmp( typestr, "Seeding" ) == 0 )
        {
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "motion_PD_para" ) ) 
            {
                // para.Kp = atof( optionfile->GetPropertyValue( prop, 0 ));
                // para.Kd = atof( optionfile->GetPropertyValue( prop, 1 ));
                // para.Err = atoi( optionfile->GetPropertyValue( prop, 2 ));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "waiting_time" ) )
            {
            }        

        }
        else if( strcmp( typestr, "Avoidance" ) == 0 )
        {
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "weight_left" ) ) 
            {
                for(int i=0;i<NUM_IRS;i++)
                    para.avoid_weightleft[i] = atoi(optionfile->GetPropertyValue(prop, i));
            }

            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "weight_right" ) )
            {
                for(int i=0;i<NUM_IRS;i++)
                    para.avoid_weightright[i] = atoi(optionfile->GetPropertyValue(prop, i));
            }        
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "weight_side" ) )
            {
                for(int i=0;i<NUM_IRS;i++)
                    para.avoid_weightside[i] = atoi(optionfile->GetPropertyValue(prop, i));
            }

        }
        else if( strcmp( typestr, "Recruitment" ) == 0 )
        {
            para.recruiting_ambient_offset1 = optionfile->ReadInt(entity, "recruiting_ambient_offset1", 500);
            para.recruiting_ambient_offset2 = optionfile->ReadInt(entity, "recruiting_ambient_offset2", 500);
            para.recruiting_proximity_offset1 = optionfile->ReadInt(entity, "recruiting_proximity_offset1", 150);
            para.recruiting_proximity_offset2 = optionfile->ReadInt(entity, "recruiting_proximity_offset2", 150);
            para.recruiting_reflective_offset1 = optionfile->ReadInt(entity, "recruiting_reflective_offset1", 20);
            para.recruiting_reflective_offset2 = optionfile->ReadInt(entity, "recruiting_reflective_offset2", 20);
        }
        else if( strcmp( typestr, "Docking" ) == 0 )
        {
            para.docking_reflective_offset1 = optionfile->ReadInt(entity, "docking_reflective_offset1", 20);
            para.docking_reflective_offset2 = optionfile->ReadInt(entity, "docking_reflective_offset2", 20);
            para.docking_reflective_diff = optionfile->ReadInt(entity, "docking_reflective_offset2", 20);
            para.docking_beacon_offset1 = optionfile->ReadInt(entity, "docking_reflective_offset1", 20);
            para.docking_beacon_offset2 = optionfile->ReadInt(entity, "docking_reflective_offset2", 20);
            para.docking_beacon_diff = optionfile->ReadInt(entity, "docking_reflective_offset2", 20);
            para.docking_motor_opening_time = optionfile->ReadInt(entity, "docking_motor_opening_time", 30);
            para.docking_motor_closing_time = optionfile->ReadInt(entity, "docking_motor_closing_time", 40);
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "turn_right_speed" ) )
            {
                para.docking_turn_right_speed[0] =  atoi(optionfile->GetPropertyValue(prop, 0));
                para.docking_turn_right_speed[1] =  atoi(optionfile->GetPropertyValue(prop, 1));
                para.docking_turn_right_speed[2] =  atoi(optionfile->GetPropertyValue(prop, 2));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "turn_left_speed" ) )
            {
                para.docking_turn_left_speed[0] =  atoi(optionfile->GetPropertyValue(prop, 0));
                para.docking_turn_left_speed[1] =  atoi(optionfile->GetPropertyValue(prop, 1));
                para.docking_turn_left_speed[2] =  atoi(optionfile->GetPropertyValue(prop, 2));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "forward_speed" ) )
            {
                para.docking_forward_speed[0] =  atoi(optionfile->GetPropertyValue(prop, 0));
                para.docking_forward_speed[1] =  atoi(optionfile->GetPropertyValue(prop, 1));
                para.docking_forward_speed[2] =  atoi(optionfile->GetPropertyValue(prop, 2));
            }
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "backward_speed" ) )
            {
                para.docking_backward_speed[0] =  atoi(optionfile->GetPropertyValue(prop, 0));
                para.docking_backward_speed[1] =  atoi(optionfile->GetPropertyValue(prop, 1));
                para.docking_backward_speed[2] =  atoi(optionfile->GetPropertyValue(prop, 2));
            }

        }
        else if( strcmp( typestr, "Locking" ) == 0 )
        {
            para.locking_proximity_offset1 = optionfile->ReadInt(entity, "locking_proximity_offset1", 720);
            para.locking_proximity_offset2 = optionfile->ReadInt(entity, "locking_proximity_offset2", 640);
            para.locking_proximity_diff1 = optionfile->ReadInt(entity, "locking_proximity_diff1", 40);
            para.locking_proximity_diff2 = optionfile->ReadInt(entity, "locking_proximity_diff1", 40);
            para.locking_reflective_offset1 = optionfile->ReadInt(entity, "locking_reflective_offset1", 500);
            para.locking_reflective_offset2 = optionfile->ReadInt(entity, "locking_reflective_offset2", 500);
            para.locking_reflective_diff = optionfile->ReadInt(entity, "locking_reflective_diff", 650);
        }
        else if( strcmp( typestr, "MacroLocomotion" ) == 0 )
        {
            para.hinge_motor_lifting_time = optionfile->ReadInt(entity, "hinge_motor_lifting_time", 30);
            para.hinge_motor_lowing_time = optionfile->ReadInt(entity, "hinge_motor_lowing_time", 40);
        }
        else if( strcmp( typestr, "Motor" ) == 0 )
        {
            para.speed_forward = optionfile->ReadInt(entity, "speed_forward", 30);
            para.speed_sideward = optionfile->ReadInt(entity, "speed_sideward", 0);

        }
        else if( strcmp( typestr, "Debugging" ) == 0 )
        {
            para.debug.mode = optionfile->ReadInt(entity, "mode", 0);

            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "para" ) )
            {
                for(int i=0;i<10;i++)
                    para.debug.para[i] =  atoi(optionfile->GetPropertyValue(prop, i));
            }

        }
        else if( strcmp( typestr, "Global" ) == 0 )
        {        
            para.logtofile = optionfile->ReadInt(entity, "logtofile", 1);
            para.init_state = optionfile->ReadInt(entity, "init_state", 0);
            para.print_proximity = optionfile->ReadInt(entity, "print_proximity", 0);
            para.print_beacon = optionfile->ReadInt(entity, "print_beacon", 0);
            para.print_ambient = optionfile->ReadInt(entity, "print_ambient", 0);
            para.print_reflective = optionfile->ReadInt(entity, "print_reflective", 0);
            para.print_status = optionfile->ReadInt(entity, "print_status", 0);
            para.ir_msg_repeated_delay = optionfile->ReadInt(entity, "ir_msg_repeated_delay", 30);
            para.ir_msg_repeated_num = optionfile->ReadInt(entity, "ir_msg_repeated_num", 10);
            para.ir_msg_ack_delay = optionfile->ReadInt(entity, "ir_msg_ack_delay", 10);
            if(para.init_state > STATE_COUNT || para.init_state <0)
                para.init_state = 0;
        }
        else if( strcmp( typestr, "ShapeInfo" ) == 0 )
        {
            if( Morph::CProperty* prop = optionfile->GetProperty( entity, "seq" ) ) 
            {
                OrganismSequence seq;
                const char *str =  optionfile->GetPropertyValue( prop, 0 );
                int size = strlen(str);
                printf("seq [%d]: ", size);
                for(int i=0;i<size;i++)
                    printf("%c", str[i]);
                printf("\n");

                rt_status ret = seq.reBuild(str);
                if(ret.status>=RT_ERROR)
                    printf("Error: Incomplete sequence record or in wrong format\n");
                else
                {
                    para.og_seq_list.push_back(seq);
                    std::cout<<seq<<std::endl;
                }

            }
        }
        else
            printf("loading something else %d\n", entity);
    }

    std::cout<<para<<std::endl;

    return true;
}

