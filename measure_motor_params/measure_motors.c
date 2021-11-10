/*******************************************************************************
* measure_motor_params.c
*   Template code 
*   Complete this code to automatically measure motor parameters
*   or print out data to be namalyzed in numpy
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/motor.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"


float enc2meters = (WHEEL_DIAMETER * M_PI) / (GEAR_RATIO * ENCODER_RES);

void test_speed(float du, float dtime_s, FILE* f_L, FILE* f_R);
float ticks_to_speed(float ticks_average, float dtime_s);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }
#endif

#if defined(BEAGLEBONE_BLUE)
    if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }
#endif

    if(rc_encoder_eqep_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
        return -1;
    }

    FILE *f_L = fopen("/home/debian/Team_212/mobilebot/measure_motor_params/data_L.txt", "w");
    if (f_L == NULL)
    {
        printf("Error opening f_L!\n");
        exit(1);
    }

    FILE *f_R = fopen("/home/debian/Team_212/mobilebot/measure_motor_params/data_R.txt", "w");
    if (f_R == NULL)
    {
        printf("Error opening f_R!\n");
        exit(1);
    }

    // Time information
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    printf ( "Current local time and date: %s", asctime (timeinfo) );
    
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	if(rc_get_state()==RUNNING){
		rc_nanosleep(1E9); //sleep for 1s
        //TODO: write routine here
	}
	
	// TODO: Plase exit routine here
    for(float pwm = 0.0; pwm <= 0.96 ;pwm += 0.05) {
        test_speed(pwm, 1, f_L, f_R); // duty, dtime_s
    }

    fclose(f_L);
    fclose(f_R);

    // cleanup
    rc_motor_set(LEFT_MOTOR, 0.0);
    rc_motor_set(RIGHT_MOTOR, 0.0);
    rc_encoder_eqep_cleanup();
    rc_motor_cleanup();
    

    // remove pid file LAST
	rc_remove_pid_file();   
	return 0;
}

void test_speed(float duty, float dtime_s, FILE* f_L, FILE* f_R){
    rc_motor_set(LEFT_MOTOR, duty * LEFT_MOTOR_POLAR);
    rc_motor_set(RIGHT_MOTOR, duty * RIGHT_MOTOR_POLAR);
    rc_nanosleep(2E8);
    rc_encoder_eqep_write(LEFT_MOTOR, 0);
    rc_encoder_eqep_write(RIGHT_MOTOR, 0);
    rc_nanosleep(dtime_s * 1E9);
    int ticks_left = rc_encoder_eqep_read(LEFT_MOTOR) * LEFT_ENCODER_POLAR;
    int ticks_right = rc_encoder_eqep_read(RIGHT_MOTOR) * RIGHT_ENCODER_POLAR;
    // float speed = ticks_to_speed((ticks_left + ticks_right) / 2, dtime_s);
    float speed_L = ticks_to_speed(ticks_left, dtime_s);
    float speed_R = ticks_to_speed(ticks_right, dtime_s);
    printf("%f, %f, %f \n", duty, speed_L, speed_R);
    fprintf(f_L, "%f,%f \n", duty, speed_L);
    fprintf(f_R, "%f,%f \n", duty, speed_R);

}

float ticks_to_speed(float ticks_average, float dtime_s) {
    return (ticks_average * enc2meters) / dtime_s;
}