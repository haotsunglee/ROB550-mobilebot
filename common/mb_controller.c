#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    mb_load_controller_config();
    left_wheel_velocity_pid = rc_filter_empty();
    right_wheel_velocity_pid = rc_filter_empty();
    low_pass = rc_filter_empty();
    // rc_filter_pid(&left_wheel_velocity_pid, 1.0, 0.0, 0.0, 0.1, DT);
    // rc_filter_pid(&right_wheel_velocity_pid, 1.0, 0.0, 0.0, 0.1, DT);
    printf("hello");
    printf("%f, %f, %f\n", pid_params.kp, pid_params.ki, pid_params.kd);
    rc_filter_pid(&left_wheel_velocity_pid, pid_params.kp, pid_params.ki, pid_params.kd, pid_params.dFilterHz, DT);
    rc_filter_pid(&right_wheel_velocity_pid, pid_params.kp, pid_params.ki, pid_params.kd, pid_params.dFilterHz, DT);
    rc_filter_enable_saturation(&left_wheel_velocity_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_velocity_pid, -1.0, 1.0);
    rc_filter_first_order_lowpass(&low_pass, DT, 0.02);
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f",
        &pid_params.kp, 
        &pid_params.ki, 
        &pid_params.kd, 
        &pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){  
    // // 1. PID controller
    // float left_error = mb_setpoints->fwd_velocity - mb_state->left_velocity;
    // float right_error = mb_setpoints->fwd_velocity - mb_state->right_velocity;
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error);
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error);

    mb_setpoints->fwd_velocity = rc_filter_march(&low_pass, mb_setpoints->fwd_velocity );
    // 1 - 2. PID controller with turning
    float velocity_r = (2.0 * mb_setpoints->fwd_velocity + mb_setpoints->turn_velocity * WHEEL_BASE) / 2.0;
    float velocity_l = (2.0 * mb_setpoints->fwd_velocity - mb_setpoints->turn_velocity * WHEEL_BASE) / 2.0;
    // printf("%f, %f", velocity_l, mb_state->left_velocity);
    float left_error = velocity_l - mb_state->left_velocity;
    float right_error = velocity_r - mb_state->right_velocity;
    mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error);
    mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error);


    // // 2. Openloop controller
    // // wheel_L:
    // // m = 1.2097
    // // b = -0.0791

    // // wheel_R:
    // // m = 1.0498
    // // b = -0.0341
    // float m_L = 1.2097;
    // float b_L = -0.0791;

    // float m_R = 1.0498;
    // float b_R = -0.0341;
    // mb_state->left_cmd = (mb_setpoints->fwd_velocity - b_L) / m_L;
    // mb_state->right_cmd = (mb_setpoints->fwd_velocity - b_R) / m_R;
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&left_wheel_velocity_pid);
    rc_filter_free(&right_wheel_velocity_pid);
    rc_filter_free(&low_pass);
    return 0;
}
