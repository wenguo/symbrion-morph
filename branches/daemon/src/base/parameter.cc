// 
//
// Author: Wenguo Liu
// Date: 02/2012
//
//
#include <string.h>
#include "parameter.hh"


Parameter::Parameter():
    logtofile(-1),
    init_state(0),
    speed_forward(30),
    speed_sideward(2),
    aw_adjustment_ratio(0.9),
    locomotion_motor_enabled(true),
    recruiting_guiding_signals_time(200),
    recruiting_beacon_signals_time(500),
    docking_trials(255),
    docking_failed_reverse_time(10),
    hinge_motor_lifting_time(30),
    hinge_motor_lowing_time(40),
    hinge_motor_speed(30),
    hinge_motor_angle(40),
    hinge_motor_default_pos(75),
    hinge_motor_enabled(true),
    locking_motor_opening_time(30),
    locking_motor_closing_time(40),
    locking_motor_isense_threshold(220),
    aligning_reverse_time(50),
    print_proximity(false),
    print_beacon(false),
    print_reflective(false),
    print_ambient(false),
    print_status(false),
    ir_msg_repeated_delay(30),
    ir_msg_repeated_num(10),
    ir_msg_ack_delay(10),
    foraging_time(80), 
    waiting_time(20),
    assembly_time(1200),
    docking_time(300),
    locatebeacon_time(300),
    fail_in_state(-1),
    fail_after_delay(0)
{
    memset(avoid_weightleft, 0, NUM_IRS * sizeof(int));
    memset(avoid_weightright, 0, NUM_IRS * sizeof(int));
    memset(avoid_weightside, 0, NUM_IRS * sizeof(int));
    memset(avoid_threshold, 0, NUM_IRS * sizeof(int));
    memset(avoid_threshold_aux, 0, NUM_IRS * sizeof(int));
    memset(beacon_threshold, 0, NUM_IRS * sizeof(int));
    memset(locatebeacon_weightleft, 0, NUM_IRS * sizeof(int));
    memset(locatebeacon_weightright, 0, NUM_IRS * sizeof(int));
    memset(aligning_weightleft, 0, NUM_IRS * sizeof(int));
    memset(aligning_weightright, 0, NUM_IRS * sizeof(int));
    memset(reflective_calibrated, 0, NUM_IRS * sizeof(int));
    memset(ambient_calibrated, 0, NUM_IRS * sizeof(int));
    memset(aux_reflective_calibrated, 0, NUM_IRS * sizeof(int));
    memset(aux_ambient_calibrated, 0, NUM_IRS * sizeof(int));
    memset(locking_motor_enabled, 0, NUM_DOCKS * sizeof(bool));
    memset(locking_motor_nonreg, 0, NUM_DOCKS * sizeof(bool));
    memset(locking_motor_opening_offset, 0, NUM_DOCKS * sizeof(int));
    memset(docking_turn_left_speed, 0, 3 * sizeof(int));
    memset(docking_turn_right_speed, 0, 3 * sizeof(int));
    memset(docking_forward_speed, 0, 3 * sizeof(int));
    memset(docking_backward_speed, 0, 3 * sizeof(int));
    memset(docking_failed_reverse_speed, 0, 3 * sizeof(int));
    memset(aligning_forward_speed, 0, 3 * sizeof(int));
    memset(aligning_reverse_speed, 0, 3 * sizeof(int));
    memset(aligning_turn_left_speed, 0, 3 * sizeof(int));
    memset(aligning_turn_right_speed, 0, 3 * sizeof(int));
    memset(locatebeacon_forward_speed, 0, 3 * sizeof(int));
    memset(locatebeacon_turn_left_speed, 0, 3 * sizeof(int));
    memset(locatebeacon_turn_right_speed, 0, 3 * sizeof(int));
    debug.mode = 0;
    memset(debug.para, 0, 10* sizeof(int));
}

Parameter::~Parameter()
{
}

std::ostream& operator<<(std::ostream& os, const Parameter& para)
{
    os  << "init_state: "<< para.init_state<<std::endl
        << "logtofile: "<< para.logtofile<<std::endl
        << "speed_forward: "<< para.speed_forward<<std::endl
        << "speed_sideward: "<< para.speed_sideward<<std::endl
        << "fail_in_state: " << para.fail_in_state << std::endl
        << "fail_after_delay: " << para.fail_after_delay << std::endl;
    os  << "avoidance_weight_left: [";
    for(int i=0;i<NUM_IRS;i++)
    {
        os << para.avoid_weightleft[i]<<" ";
    }
    os <<"]\n";
    os  << "avoidance_weight_right: [";
    for(int i=0;i<NUM_IRS;i++)
    {
        os << para.avoid_weightright[i]<<" ";
    }
    os <<"]\n";
    os  << "avoidance_weight_side: [";
    for(int i=0;i<NUM_IRS;i++)
    {
        os << para.avoid_weightside[i]<<" ";
    }
    os <<"]\n";
    os  << "ambient_calibrated: [";
    for(int i=0;i<NUM_IRS;i++)
    {
        os << para.ambient_calibrated[i]<<" ";
    }
    os <<"]\n";
    os  << "reflective_calibrated: [";
    for(int i=0;i<NUM_IRS;i++)
    {
        os << para.reflective_calibrated[i]<<" ";
    }
    os <<"]\n";





    return os;

}
