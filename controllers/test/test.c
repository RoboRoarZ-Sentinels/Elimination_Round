#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

// --- Constants ---
#define MAX_SPEED 6.0
#define TIME_STEP 10

// User Request: Radius 0.0201 meters (2.01 cm)
#define WHEEL_RADIUS 0.0201 

// User Request: Cell distance 25 cm
#define CELL_DISTANCE 25.0 

// --- Global Device Tags ---
WbDeviceTag left_ps, right_ps, left_motor, right_motor;
WbDeviceTag ds_left, ds_right; 
WbDeviceTag ds_front_left, ds_front_right; // Two front sensors

// --- Helper Functions ---

void wait_seconds(double seconds) {
    double start_time = wb_robot_get_time();
    while (wb_robot_get_time() < start_time + seconds) {
        wb_robot_step(TIME_STEP);
    }
}

int API_moveForward() {
    printf("Debug: Starting Move Forward (Target: 25cm)...\n");
    
    // Step 1: Read initial encoder values
    double left_start = wb_position_sensor_get_value(left_ps);
    double right_start = wb_position_sensor_get_value(right_ps);
    
    // PID Constants (Tune these if the robot wobbles)
    double Kp = 0.001;      
    double Ki = 0.000000;   
    double Kd = 0.04;       

    double previous_error = 0;
    double integral = 0;
    double error = 0;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // --- 1. Distance Calculation ---
        double left_enc = wb_position_sensor_get_value(left_ps) - left_start;
        double right_enc = wb_position_sensor_get_value(right_ps) - right_start;
        
        // Convert radians to cm:  (Radians * Radius_in_meters * 100)
        double left_dist_cm = left_enc * (WHEEL_RADIUS * 100.0);
        double right_dist_cm = right_enc * (WHEEL_RADIUS * 100.0);

        // --- 2. Sensor Readings ---
        double left_val = wb_distance_sensor_get_value(ds_left);
        double right_val = wb_distance_sensor_get_value(ds_right);
        
        // Get values from both front sensors
        double front_left_val = wb_distance_sensor_get_value(ds_front_left);
        double front_right_val = wb_distance_sensor_get_value(ds_front_right);
        
        // Calculate Average
        double front_avg = (front_left_val + front_right_val) / 2.0;

        // --- 3. PID Logic (Wall Centering) ---
        double derivative = 0;
        double correction = 0;
        
        // Both walls detected
        if (left_val < 165 && right_val < 165) {
            error = right_val - left_val;
            integral += error;
            derivative = (error - previous_error);
            correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        // Only right wall detected
        else if (left_val >= 165 && right_val < 165) {
            error = right_val - 150; // Maintain distance 700
            integral += error;
            derivative = (error - previous_error);
            correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        // Only left wall detected
        else if (left_val < 165 && right_val >= 165) {
            error = 150 - left_val; // Maintain distance 700
            integral += error;
            derivative = (error - previous_error);
            correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        else {
            correction = 0;
        }
        
        previous_error = error;

        // Apply Motor Speeds
        double left_speed = (MAX_SPEED/2) + correction;
        double right_speed = (MAX_SPEED/2) - correction;

        // Cap speeds
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
        if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
        if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);


        if (front_avg < 155) { 
            printf("Debug: Front Wall Detected (Avg: %.2f)! Stopping.\n", front_avg);
            break;
        }
        // Target distance reached
        else if (left_dist_cm >= CELL_DISTANCE && right_dist_cm >= CELL_DISTANCE) {
            printf("Debug: Target distance %.2f cm reached.\n", CELL_DISTANCE);
            break;
        }
    }
 
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
    
    return 1;
}

int main(int argc, char **argv) {
    wb_robot_init();

    printf("--- Initializing Move Forward Test (2 Front Sensors) ---\n");

    // Initialize Motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    // Initialize Side Sensors
    ds_left = wb_robot_get_device("ps5");
    ds_right = wb_robot_get_device("ps2");
    

    ds_front_left = wb_robot_get_device("ps7");
    ds_front_right = wb_robot_get_device("ps0");

    wb_distance_sensor_enable(ds_left, TIME_STEP);
    wb_distance_sensor_enable(ds_right, TIME_STEP);
    wb_distance_sensor_enable(ds_front_left, TIME_STEP);
    wb_distance_sensor_enable(ds_front_right, TIME_STEP);
    
    // Initialize Encoders
    right_ps = wb_robot_get_device("right wheel sensor");
    left_ps = wb_robot_get_device("left wheel sensor");

    wb_position_sensor_enable(right_ps, TIME_STEP);
    wb_position_sensor_enable(left_ps, TIME_STEP);

    wait_seconds(0.5);

    while (wb_robot_step(TIME_STEP) != -1) {
        
        API_moveForward();

        printf("Stopped. Waiting 1 second...\n");
        wait_seconds(1.0);
    }

    wb_robot_cleanup();
    return 0;
}