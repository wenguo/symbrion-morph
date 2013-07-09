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

        int logtofile;

        int init_state;
        int speed_forward;
        int speed_sideward;
        float aw_adjustment_ratio;
        bool locomotion_motor_enabled;
        int scout_wheels_direction[2];
        int locatebeacon_forward_speed[3];
        int locatebeacon_turn_left_speed[3];
        int locatebeacon_turn_right_speed[3];
        int aligning_forward_speed[3];
        int aligning_reverse_speed[3];
        int aligning_turn_left_speed[3];
        int aligning_turn_right_speed[3];
        int docking_turn_left_speed[3];
        int docking_turn_right_speed[3];
        int docking_forward_speed[3];
        int docking_backward_speed[3];
        int docking_failed_reverse_speed[3];
        int recruiting_guiding_signals_time;
        int recruiting_beacon_signals_time;

        int docking_trials;
        int docking_failed_reverse_time;
        int hinge_motor_lifting_time;
        int hinge_motor_lowing_time;
        int hinge_motor_speed;
        int hinge_motor_angle;
        int hinge_motor_default_pos; //
        bool hinge_motor_enabled;

        int locking_motor_opening_time;
        int locking_motor_closing_time;
        bool locking_motor_enabled[NUM_DOCKS];
        int locking_motor_isense_threshold;
        int locking_motor_opening_offset[NUM_DOCKS];
        bool locking_motor_nonreg[NUM_DOCKS];

        int avoid_weightleft[NUM_IRS];
        int avoid_weightright[NUM_IRS];
        int avoid_weightside[NUM_IRS];
        int avoid_threshold[NUM_IRS];
        int avoid_threshold_aux[NUM_IRS];

        int beacon_threshold[NUM_IRS];

        int locatebeacon_weightleft[NUM_IRS];
        int locatebeacon_weightright[NUM_IRS];

        int aligning_weightleft[NUM_IRS];
        int aligning_weightright[NUM_IRS];

        int aligning_reverse_time;

        bool print_proximity;
        bool print_beacon;
        bool print_reflective;
        bool print_ambient;
        bool print_status;

        uint32_t ir_msg_repeated_delay;
        uint32_t ir_msg_repeated_num;
        uint32_t ir_msg_ack_delay;

        int ambient_calibrated[NUM_IRS];
        int reflective_calibrated[NUM_IRS];
        int aux_ambient_calibrated[NUM_IRS];
        int aux_reflective_calibrated[NUM_IRS];

        int foraging_time;
        int waiting_time;
        int assembly_time;
        int locatebeacon_time;
        int docking_time;

        int fail_in_state;
        int fail_after_delay;

        std::vector<OrganismSequence> og_seq_list;

        struct{
            int mode;
            int para[10];
        } debug;

};

#endif
