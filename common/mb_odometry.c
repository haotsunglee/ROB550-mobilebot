/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846
#define THRESHOLD 0.000125

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    float enc2meters = (WHEEL_DIAMETER * PI) / (GEAR_RATIO * ENCODER_RES);
    float delta_d, delta_theta, delta_x, delta_y, delta_theta_odo, delta_theta_gyro;
    delta_d = enc2meters * (mb_state->left_encoder_delta + mb_state->right_encoder_delta) / 2.0;
    delta_theta_odo = enc2meters * (mb_state->right_encoder_delta - mb_state->left_encoder_delta) / WHEEL_BASE;
    delta_theta_gyro = mb_angle_diff_radians(mb_state->last_yaw, mb_state->tb_angles[2]);
    float delta_g_o = delta_theta_gyro - delta_theta_odo;
    if(fabs(delta_g_o) > THRESHOLD){
        delta_theta = delta_theta_gyro;
    }else{
        delta_theta = delta_theta_odo;
    }

    delta_x = delta_d * cos(mb_odometry->theta + delta_theta / 2.0);
    delta_y = delta_d * sin(mb_odometry->theta + delta_theta / 2.0);
    mb_odometry->x += delta_x;
    mb_odometry->y += delta_y;
    mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + delta_theta);

    mb_state -> turn_velocity = delta_theta / DT;
    mb_state -> fwd_velocity = delta_d / DT;

}


/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -PI)
    {
        for(; angle < -PI; angle += 2.0*PI);
    }
    else if(angle > PI)
    {
        for(; angle > PI; angle -= 2.0*PI);
    }

    return angle;
}


/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}