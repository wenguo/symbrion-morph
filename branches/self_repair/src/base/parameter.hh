// 
//
// Author: Wenguo Liu
// Date: 02/2012
//
//
#ifndef PARAMETER_HH
#define PARAMETER_HH

#include <iostream>
#include "global.hh"
#include "og/organism.hh"

class Parameter
{
    public:
        Parameter();
        ~Parameter();

        friend std::ostream& operator<<(std::ostream&, const Parameter&);

        bool logtofile;

        int init_state;
        int speed_forward;
        int speed_sideward;
        int docking_turn_left_speed[3];
        int docking_turn_right_speed[3];
        int docking_forward_speed[3];
        int docking_backward_speed[3];
        int recruiting_ambient_offset1;
        int recruiting_ambient_offset2;
        int recruiting_proximity_offset1;
        int recruiting_proximity_offset2;
        int recruiting_reflective_offset1;
        int recruiting_reflective_offset2;

        int docking_reflective_offset1;
        int docking_reflective_offset2;
        int docking_reflective_diff;
        int docking_beacon_offset1;
        int docking_beacon_offset2;
        int docking_beacon_diff;
        int docking_motor_opening_time;
        int docking_motor_closing_time;
        int hinge_motor_lifting_time;
        int hinge_motor_lowing_time;

        int locking_proximity_offset1;
        int locking_proximity_offset2;
        int locking_proximity_diff1;
        int locking_proximity_diff2;
        int locking_reflective_offset1;
        int locking_reflective_offset2;
        int locking_reflective_diff;
        bool locking_motor_enabled[NUM_DOCKS];

        int avoid_weightleft[NUM_IRS];
        int avoid_weightright[NUM_IRS];
        int avoid_weightside[NUM_IRS];

        bool print_proximity;
        bool print_beacon;
        bool print_reflective;
        bool print_ambient;
        bool print_status;

        uint32_t ir_msg_repeated_delay;
        uint32_t ir_msg_repeated_num;
        uint32_t ir_msg_ack_delay;

        int32_t avoidance_threshold;


        std::vector<OrganismSequence> og_seq_list;

        struct{
            int mode;
            int para[10];
        } debug;

};

#endif
