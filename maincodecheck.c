
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <stdio.h>

#define MAX_SPEED 10.0
#define TARGET_DISTANCE 23  // Distance in cm

#define TIME_STEP 10
#define WHEEL_RADIUS 0.025  // Adjust according to your robot
#define AXLE_LENGTH 0.11


#include <stdlib.h>
#include <string.h>

#ifndef CONSTANTS_H
#define CONSTANTS_H

WbDeviceTag left_ps, right_ps, left_motor, right_motor, ds_left, ds_right,ds_front,cameraright, cameraleft, cameradown;

int num = 0;
int LOWER_X_GOAL = 10;
int LOWER_Y_GOAL = 10;
int UPPER_X_GOAL = 10;
int UPPER_Y_GOAL = 10;

int RESET_AT_CENTER = 0;

// floodfill weighting scores
const int TURN_SCORE = 5;
const int TILE_SCORE0 = 10;
const int TILE_SCORE1 = 5000;
const int TILE_SCORE2 = 10000;
const int TILE_SCORE3 = 20000;
const int STREAK_SCORE = -1;
const int STREAK_MULTIPLIER = -1;


// if you want the mouse to reset to the start once it reaches the middle, set RESET_AT_CENTER to 1
// otherwise, set it to 0 if you want it to make its way back to the start

const unsigned char STAY_AT_CENTER = 1;

// starting position. keep it the same for all regular micromouse rules
#define STARTING_TARGET 1; // 0 if going to start, 1 if going to center
#define STARTING_HEADING NORTH;
#define STARTING_X 10
#define STARTING_Y 0


// goal position, assuming the goal is a square
// when setting this, remember that the maze is zero-indexed!


// maze size
#define MAZE_WIDTH 20
#define MAZE_HEIGHT 20

// highlight path
const unsigned char HIGHLIGHT_PATH = 1;

// DO NOT TOUCH UNLESS YOU KNOW WHAT YOU ARE DOING
#define OUT_OF_BOUNDS -2
#define NOT_YET_SET -1

typedef enum Heading {NORTH, WEST, SOUTH, EAST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE} Action;

typedef struct coord {
    int x;
    int y;
} coord;

typedef struct neighbor {
    coord coord;
    Heading heading;
    int streak;
} neighbor;

Action solver();
void generateInitialWalls();



typedef neighbor item_type;
typedef struct _queue* queue;
queue queue_create();
void queue_destroy(queue q);
void queue_push(queue q, item_type elem);
item_type queue_pop(queue q);
item_type queue_first(queue q);
int queue_is_empty(queue q);
int queue_size(queue q);
void queue_clear(queue q);

// queue type implementation

struct node {
    item_type data;
    struct node* next;
};


int API_wallFront();
int API_wallRight();
int API_wallLeft();
int detectgreen();
int detectfloor();
int API_moveForward();  // Returns 0 if crash, else returns 1
void API_turnRight();
void API_turnLeft();
void debug_log(char* text);


#endif

struct _queue {
    struct node* head;
    struct node* tail;
    int size;
};

queue queue_create() {
queue q = (queue) malloc(sizeof(struct _queue));
    if (q == NULL) {
        fprintf(stderr, "Insufficient memory to \
        initialize queue.\n");
        abort();
    }
    q->head = NULL;
    q->tail = NULL;
    q->size = 0;
    return q;
}

void queue_destroy(queue q) {
    if (q == NULL) {
        fprintf(stderr, "Cannot destroy queue\n");
        abort();
    }
    queue_clear(q);
    free(q);
}

void queue_push(queue q, item_type elem) {
    struct node* n;
    n = (struct node*) malloc(sizeof(struct node));
    if (n == NULL) {
        fprintf(stderr, "Insufficient memory to \
        create node.\n");
        abort();
    }
    n->data = elem;
    n->next = NULL;
    if (q->head == NULL) {
        q->head = q->tail = n;
    } else {
        q->tail->next = n;
        q->tail = n;
    }
    q->size += 1;
}

item_type queue_pop(queue q) {
    if (queue_is_empty(q)) {
    fprintf(stderr, "Can't pop element from queue: \
    queue is empty.\n");
    abort();
    }
    struct node* head = q->head;
    if (q->head == q->tail) {
        q->head = NULL;
        q->tail = NULL;
    } else {
        q->head = q->head->next;
    }
    q->size -= 1;
    item_type data = head->data;
    free(head);
    return data;
}

item_type queue_first(queue q) {
    if (queue_is_empty(q)) {
        fprintf(stderr, "Can't return element from queue: \
        queue is empty.\n");
        abort();
    }
   
    return q->head->data;
}

int queue_is_empty(queue q) {
    if (q==NULL) {
        fprintf(stderr, "Cannot work with NULL queue.\n");
        abort();
    }
    return q->head == NULL;
}

int queue_size(queue q) {
    if (q==NULL) {
        fprintf(stderr, "Cannot work with NULL queue.\n");
        abort();
    }
    return q->size;
}

void queue_clear(queue q) {
    if (q==NULL) {
        fprintf(stderr, "Cannot work with NULL queue.\n");
        abort();
    }
    while(q->head != NULL) {
        struct node* tmp = q->head;
        q->head = q->head->next;
        free(tmp);
    }
    q->tail = NULL;
    q->size = 0;
}



unsigned char target = STARTING_TARGET; 
coord currentXY = {STARTING_X, STARTING_Y};
Heading currentHeading = STARTING_HEADING;

/* Arrays and Array Helper Functions */

// keeps track of the known vertical walls in the maze
int verticalWalls[MAZE_WIDTH+1][MAZE_HEIGHT] = {{0}};
// keeps track of horizontal walls in the maze
int horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT+1] = {{0}};
// keeps track of current floodfill values
int floodArray[MAZE_WIDTH][MAZE_HEIGHT];
// keeps track of the path the car should take after a floodfill iteration for each cell of the maze
Heading pathArray[MAZE_WIDTH][MAZE_HEIGHT] = {{NORTH}};
// keeps track of all of the cells that the mouse has visited
int travelArray[MAZE_WIDTH][MAZE_HEIGHT] = {{0}};
int survivorArray[MAZE_WIDTH][MAZE_HEIGHT] = {{0}};
int fireArray[MAZE_WIDTH][MAZE_HEIGHT] = {{0}};

// given a coord, checks to see if the mouse has visited a certain cell before
int checkTravelArray(coord c) {return travelArray[c.x][c.y];}
// given a coord, updates the travel array to mark that the mouse has visited that cell before
void updateTravelArray(coord c) {travelArray[c.x][c.y] = 1;}

void updatesurvivorArray(coord c) {survivorArray[c.x][c.y] = 1;}
// given coordinate, updates the respective cell's floodfill value
void updateFloodArray(coord c, int val) {floodArray[c.x][c.y] = val;}
// given coordinate, gets the respective cell's floodfill value
int getFloodArray(coord c) {return floodArray[c.x][c.y];}
// given coordinate, updates the respective cell's path heading
void updatePathArray(coord c, Heading h) {pathArray[c.x][c.y] = h;}
// given cordinate, gets the respective cell's path heading
Heading getPathArray(coord c) {return pathArray[c.x][c.y];}


void updatefirelArray(coord c, int val) {fireArray[c.x][c.y] = val;}
int checkfireArray(coord c){return fireArray[c.x][c.y];}

/* Floodfill Functions */

// resets the floodfill array to target the center as destination
void resetFloodArray()
{
    // set the entire flood array to blank values (-1)
    for (int x = 0; x < MAZE_WIDTH; x++)
        for (int y = 0; y < MAZE_HEIGHT; y++)
            floodArray[x][y] = -1;
    // set desired goal values 
    if (target) // target is goal (center)
        for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
            for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++)
                floodArray[x][y] = 0;
    else // target is starting cell
        floodArray[STARTING_X][STARTING_Y] = 0;
}

// given heading and coordinate, check if there is a wall on that side of the cell
int checkWall(Heading heading, coord c) {
    switch (heading) {
        case NORTH: return horizontalWalls[c.x][c.y+1];
        case WEST: return verticalWalls[c.x][c.y];
        case SOUTH: return horizontalWalls[c.x][c.y];
        case EAST: return verticalWalls[c.x+1][c.y];
    }
}

// Increments coord in the direction of the heading by input integer, then returns updated coord
coord incrementCoord(Heading heading, coord c, int numCells) {
    switch (heading) {
        case NORTH: return (coord){c.x, c.y += numCells};
        case WEST: return (coord){c.x -= numCells, c.y};
        case SOUTH: return (coord){c.x, c.y -= numCells};
        case EAST: return (coord){c.x += numCells, c.y};
    }
}

// turns currentHeading global variable to the left based on the mouse's current heading,
// then returns LEFT action
Action turnLeft() {
    API_turnLeft();
    currentHeading = (currentHeading+1)%4;
    return LEFT;
}

// turns currentHeading global variable to the right based on the mouse's current heading,
// then returns RIGHT action
Action turnRight() {
    API_turnRight();
    currentHeading = (currentHeading-1)%4;
    return RIGHT;
}

// returns whether the mouse is in the target
unsigned char mouseInGoal() {
    return (target == 1 && (currentXY.x >= LOWER_X_GOAL && currentXY.x <= UPPER_X_GOAL && currentXY.y >= LOWER_Y_GOAL && currentXY.y <= UPPER_Y_GOAL));
}

// given heading and coordinates, returns the floodfill value of the corresponding neighbor cell.
// if the neighbor is off of the maze (argument cell is on the boundary of the maze), return -2
int getNeighbor(Heading heading, coord c)
{
    switch (heading) {
        case NORTH:
            if (c.y >= 19) return OUT_OF_BOUNDS;
            else return floodArray[c.x][c.y+1];
        case WEST:
            if (c.x <= 0) return OUT_OF_BOUNDS;
            else return floodArray[c.x-1][c.y];
        case SOUTH:
            if (c.y <= 0) return OUT_OF_BOUNDS;
            else return floodArray[c.x][c.y-1];
        case EAST:
            if (c.x >= 19) return OUT_OF_BOUNDS;
            else return floodArray[c.x+1][c.y];
    }
}

neighbor generateNeighbor(queue q, Heading heading, neighbor current, int currentVal) {
    if (!checkWall(heading,current.coord)) {
        int nextVal = 0;
        if (checkfireArray(current.coord ) == 3){
             nextVal = currentVal + TILE_SCORE3;
        }
        else if (checkfireArray(current.coord ) == 2){
             nextVal = currentVal + TILE_SCORE2;
        }
        else if (checkfireArray(current.coord ) == 1){
             nextVal = currentVal + TILE_SCORE1;
        }
        else {
             nextVal = currentVal + TILE_SCORE0;
        }
        neighbor next;
        // checks if the mouse would have to turn to go north from current cell
        if (current.heading != heading) {
            nextVal += TURN_SCORE;
            next.streak = 0;
        } else { // if the mouse doesn't need to turn, records that is is on a straight streak
            nextVal += (STREAK_MULTIPLIER * (current.streak-1)) + STREAK_SCORE;
            next.streak = current.streak + 1;
        }

        // prepare neighbor to add to the floodfill queue
        next.coord = incrementCoord(heading, current.coord, 1);
        next.heading = heading;

        int neighborVal = getNeighbor(heading,current.coord);
        if (neighborVal == NOT_YET_SET || nextVal < neighborVal) {
            queue_push(q,next);
            updateFloodArray(next.coord,nextVal);
            updatePathArray(next.coord,(heading+2)%4);
        }
    }
}

// updates the floodfill array based on known walls
void floodFill() {
    // set non-goal values to blank so that the floodfill array can be recalculated
    resetFloodArray();

    // declare/initialize relevant variables for queue for floodfill algorithm
    queue q = queue_create();

    // iterate through the 2D array, find goal values and add them to the queue
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            if (floodArray[x][y] == 0) {
                // for the starting goal values, it doesn't matter which direction you approach them from.
                // as such, they should be oriented from all directions
                queue_push(q,(neighbor){(coord){x,y},NORTH,0});
                queue_push(q,(neighbor){(coord){x,y},WEST,0});
                queue_push(q,(neighbor){(coord){x,y},SOUTH,0});
                queue_push(q,(neighbor){(coord){x,y},EAST,0});
            }
        }
    }

    // adds available neighbors to queue and updates their floodfill values
    while (!queue_is_empty(q)) {
        // initializes values for calculating floodfills for neighbors
        neighbor current = queue_pop(q);
        int currentVal = getFloodArray(current.coord);

        // prints the current cell's floodfill number to the simulation screen
        
        // pushes neighbors if available
        generateNeighbor(q,NORTH,current,currentVal);
        generateNeighbor(q,WEST,current,currentVal);
        generateNeighbor(q,SOUTH,current,currentVal);
        generateNeighbor(q,EAST,current,currentVal);        
    }
}

// places a wall in respective arrays and API at the given heading and coordinate
void placeWall(Heading heading, coord c) {
    // sets a wall in the wall arrays
    switch (heading) {
        case NORTH:
            horizontalWalls[c.x][c.y+1] = 1;
            
            return;
        case WEST:
            verticalWalls[c.x][c.y] = 1;
            
            return;
        case SOUTH:
            horizontalWalls[c.x][c.y] = 1;
            
            return;
        case EAST:
            verticalWalls[c.x+1][c.y] = 1;
            
            return;
    }
}

void generateInitialWalls() {
    for (int x = 0; x < MAZE_WIDTH; x++) {
        placeWall(SOUTH,(coord){x,0});
        placeWall(NORTH,(coord){x,MAZE_HEIGHT-1});
    }
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        placeWall(WEST,(coord){0,y});
        placeWall(EAST,(coord){MAZE_WIDTH-1,y});
    }
}

// checks for and then updates the walls for the current cell
void updateWalls()
{
    // based on the current heading, places walls at the respective locations
     if (API_wallFront()) placeWall(currentHeading,currentXY);
    if (API_wallLeft()) placeWall((currentHeading+1)%4,currentXY);
    if (API_wallRight()) placeWall((currentHeading-1)%4,currentXY);
}

// based on updated wall and floodfill information, return the next action that the mouse should do
Action nextAction()
{
    // stay at center if already in center
    if (target && mouseInGoal() && STAY_AT_CENTER)
        return IDLE;

    Heading newHeading = getPathArray(currentXY);
    updateTravelArray(currentXY);
    if (detectgreen()){
        updatesurvivorArray(currentXY);
    }
    
    int floor_col = detectfloor();
    updatefirelArray(currentXY, floor_col);

    
    coord originalCoord = currentXY;

    // moves forward if the mouse is already facing the correct heading
    if (newHeading == currentHeading) {
        //int moveNumber = 0;
        while (checkTravelArray(currentXY) == 1 && (!checkWall(newHeading,currentXY))
        && getPathArray(currentXY) == currentHeading) {
            //moveNumber++;
            updateTravelArray(currentXY);
            currentXY = incrementCoord(newHeading,currentXY,1);
            //printf("move_forward");
            API_moveForward();
        } 
       
        return FORWARD;
    }

    // determines which way to turn based on current direction and desired direction
    if (currentHeading == (newHeading+1)%4){
        //printf("turn right:");
        return turnRight();
    }
    
    else if (currentHeading == (newHeading-1)%4){
        //printf("turn left:");
        return turnLeft();
    }
    
    else {
        //debug_log("turned 180");
        turnLeft();
        return turnLeft();
    }
}
// checks if the mouse has reached its target
void checkDestination()
{
    if (target) {
        if (mouseInGoal()) {
            num += 1;
            if (RESET_AT_CENTER) {
                
                currentXY = (coord){0,0};
                currentHeading = NORTH;
            }
            else if (!STAY_AT_CENTER)
                target = 0;       
        }
    } else if (currentXY.x == STARTING_X && currentXY.y == STARTING_Y)
        target = 1;
}


// sends the mouse's recommended next action back to main
Action solver() {
    checkDestination();
    updateWalls();    
    floodFill();
    return nextAction();
}





int API_wallLeft() {
    
    return wb_distance_sensor_get_value(ds_left) < 2000;
}

int API_wallRight() {
    
    return wb_distance_sensor_get_value(ds_right) < 2000;
}

int API_wallFront() {
    return wb_distance_sensor_get_value(ds_front) < 1500;
    
}

int API_moveForward(){
    // Step 1: Read initial encoder values
    double left_start = wb_position_sensor_get_value(left_ps);
    double right_start = wb_position_sensor_get_value(right_ps);
    double target_distance = 24;
    
    double Kp = 0.001;  // Proportional gain
    double Ki = 0.000000; // Integral gain
    double Kd = 0.04 ; // Derivative gain

    double previous_error = 0;
    double integral = 0;
    double error =0;
    double ini_left_walldistance = wb_distance_sensor_get_value(ds_left);
    double ini_right_walldistance = wb_distance_sensor_get_value(ds_right);
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Compute relative encoder values
        double left_enc = wb_position_sensor_get_value(left_ps) - left_start;
        double right_enc = wb_position_sensor_get_value(right_ps) - right_start;
        // Convert encoder values to distance
        double left_dist = left_enc * 2.5;
        double right_dist = right_enc * 2.5;
        double left_walldistance = wb_distance_sensor_get_value(ds_left);
        double right_walldistance = wb_distance_sensor_get_value(ds_right);
        double front_walldistance = wb_distance_sensor_get_value(ds_front);
        double derivative =0;
        double correction =0;
        
        if (left_walldistance < 1000 && right_walldistance < 1000){
          error =  right_walldistance - left_walldistance;
          integral += error;
         derivative = (error - previous_error);
         correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        else if (left_walldistance >= 1000 && right_walldistance < 1000){
          error =  right_walldistance - 700;
          integral += error;
         derivative = (error - previous_error);
         correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        else if (left_walldistance < 1000 && right_walldistance >= 1000){
          error =  700 - left_walldistance;
          integral += error;
         derivative = (error - previous_error);
         correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
        else{
          correction = 0;
        }
        
    
        previous_error = error;

        // Motor speed adjustment
        double left_speed = MAX_SPEED + correction;
        double right_speed =  MAX_SPEED- correction;

        // Limit speed
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
        if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
        if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

        // Set motor speeds
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        if (front_walldistance < 700){
          break;
        }
        // Stop when the target distance is reached
        else if (left_dist >= target_distance && right_dist >= target_distance) {
            
            break;
        }
    }
 
   
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
    double start_time = wb_robot_get_time();  // Get current simulation time
    // while (wb_robot_get_time() - start_time < 0.5) {
        // wb_robot_step(TIME_STEP);  // Keep stepping until 3 seconds pass
    // }
    return 1;
}




void API_turnLeft() {
    const double angle = M_PI / 2;  // 90 degrees in radians
    const double speed = 5.0;
    const double turnerror = 0;
    
    // Compute required wheel rotation in radians
    double wheel_rotation = ((angle * AXLE_LENGTH)+turnerror) / (2 * WHEEL_RADIUS);

    // Get initial encoder values after a small delay to ensure valid readings
    double initial_left_pos = wb_position_sensor_get_value(left_ps);
    double initial_right_pos = wb_position_sensor_get_value(right_ps);


    // Set wheel speeds for turning
    wb_motor_set_velocity(left_motor, -speed);
    wb_motor_set_velocity(right_motor, speed);

    // Monitor sensor values to determine when to stop
    while (wb_robot_step(TIME_STEP) != -1) {
        double left_pos = wb_position_sensor_get_value(left_ps);
        double right_pos = wb_position_sensor_get_value(right_ps);

        double left_moved = left_pos - initial_left_pos;
        double right_moved = initial_right_pos - right_pos;  

        // Check if the robot has turned the desired amount
        if (fabs(left_moved) >= wheel_rotation && fabs(right_moved) >= wheel_rotation) {
            break;  // Stop when the required rotation is reached
        }
    }

    // Stop the motors
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void API_turnRight() {
    const double angle = M_PI / 2;  // 90 degrees in radians
    const double speed = 5.0;
    const double turnerror = 0;
    
    // Compute required wheel rotation in radians
    double wheel_rotation = ((angle * AXLE_LENGTH)+turnerror) / (2 * WHEEL_RADIUS);

    // Get initial encoder values after a small delay to ensure valid readings
    double initial_left_pos = wb_position_sensor_get_value(left_ps);
    double initial_right_pos = wb_position_sensor_get_value(right_ps);

    // Set wheel speeds for turning right (left wheel forward, right wheel backward)
    wb_motor_set_velocity(left_motor, speed);
    wb_motor_set_velocity(right_motor, -speed);

    // Monitor sensor values to determine when to stop
    while (wb_robot_step(TIME_STEP) != -1) {
        double left_pos = wb_position_sensor_get_value(left_ps);
        double right_pos = wb_position_sensor_get_value(right_ps);

        double left_moved = left_pos - initial_left_pos;
        double right_moved = initial_right_pos - right_pos;  

       
        if (fabs(left_moved) >= wheel_rotation && fabs(right_moved) >= wheel_rotation) {
            break;  // Stop when the required rotation is reached
        }
    }

    // Stop the motors
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

int detectgreen(){
    const unsigned char *image1 = wb_camera_get_image(cameraleft);
    const unsigned char *image2 = wb_camera_get_image(cameraright);
    if (!image1) return 0;  // No image captured

    int width = wb_camera_get_width(cameraleft);
    int height = wb_camera_get_height(cameraleft);

    int found_green = 0;

    int center_x = width / 2;
    int center_y = height / 2;
    int search_range = 10;  // Increase the range to check more pixels


    for (int dx = -search_range; dx <= search_range; dx++) {
        for (int dy = -search_range; dy <= search_range; dy++) {
            int x = center_x + dx;
            int y = center_y + dy;

            if (x < 0 || x >= width || y < 0 || y >= height) continue;  // Ensure valid pixel range

            int r = wb_camera_image_get_red(image1, width, x, y);
            int g = wb_camera_image_get_green(image1, width, x, y);
            int b = wb_camera_image_get_blue(image1, width, x, y);
            
            int r2 = wb_camera_image_get_red(image2, width, x, y);
            int g2 = wb_camera_image_get_green(image2, width, x, y);
            int b2 = wb_camera_image_get_blue(image2, width, x, y);
            
            // printf("✅ Green detected at pixel (%d, %d) - RGB(%d, %d, %d)\n", x, y, r, g, b);
            // printf("✅ Green detected at pixel (%d, %d) - RGB(%d, %d, %d)\n", x, y, r2, g2, b2);

            // Adjust the green detection threshold dynamically based on actual RGB values
            if (r < 120 && g > 50 && b < 50) {
                found_green = 1;
                printf("✅ Green detected at pixel (%d, %d) - RGB(%d, %d, %d)\n", x, y, r, g, b);
                return 1;  // Stop searching once green is detected
            }
            if (r2 < 120 && g2 > 50 && b2 < 50) {
                found_green = 1;
                printf("✅ Green detected at pixel (%d, %d) - RGB(%d, %d, %d)\n", x, y, r2, g2, b2);
                return 1;  // Stop searching once green is detected
            }
        }
    }

    return found_green;
}

int detectfloor(){
    
     const unsigned char *image = wb_camera_get_image(cameradown);
    if (!image) {
        printf("No floor image captured, defaulting to white.\n");
        return 0;  // If there's no image, treat it as white
    }
    
    // Get center pixel
    int width = wb_camera_get_width(cameradown);
    int height = wb_camera_get_height(cameradown);
    int center_x = width / 2;
    int center_y = height / 2;

    // Extract RGB from the center pixel
    int r = wb_camera_image_get_red(image, width, center_x, center_y);
    int g = wb_camera_image_get_green(image, width, center_x, center_y);
    int b = wb_camera_image_get_blue(image, width, center_x, center_y);

    
    // White: R>200, G>200, B>200
    if (r > 60 && g > 80 && b > 100) {
        //printf(" Floor Detected: **white**\n");
        return 0; // White
    }
    // Yellow: R>200, G>200, B<150
    else if (r > 30 && g > 60 && b > 40 && b < 100) {
        printf("🟡 Floor Detected: Yellow\n");
        return 1; // Yellow
    }
    // Orange: R>200, G>80..200, B<80
    else if (r > 30 && g > 20  && g < 60 && b < 40) {
        printf("🟠 Floor Detected: Orange\n");
        return 2; // Orange
    }
    // Red: R>200, G<100, B<100
    if (r > 50 && g < 20  && b < 40) {
        printf("🔴 Floor Detected: Red\n");
        return 3; // Red
    }
}


void debug_log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}





int main(int argc, char **argv) {
    wb_robot_init();

    // Initialize motors
    left_motor = wb_robot_get_device("LeftMotor");
    right_motor = wb_robot_get_device("RightMotor");
    
    cameraright = wb_robot_get_device("camera");
    cameraleft = wb_robot_get_device("camera(1)");
    cameradown = wb_robot_get_device("camera(2)");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    // Initialize sensors
    ds_left = wb_robot_get_device("ds_left");
    ds_right = wb_robot_get_device("ds_right");
    ds_front = wb_robot_get_device("ds_front");

    wb_distance_sensor_enable(ds_left, TIME_STEP);
    wb_distance_sensor_enable(ds_right, TIME_STEP);
    wb_distance_sensor_enable(ds_front, TIME_STEP);
    
    wb_camera_enable(cameraleft, TIME_STEP);
    wb_camera_enable(cameraright, TIME_STEP);
    wb_camera_enable(cameradown, TIME_STEP);
    // Initialize position sensors
    right_ps = wb_robot_get_device("ps_right");
    left_ps = wb_robot_get_device("ps_left");

    wb_position_sensor_enable(right_ps, TIME_STEP);
    wb_position_sensor_enable(left_ps, TIME_STEP);

    // Compute encoder unit
    double wheel_radius = 0.025;  // 2.5 cm
    double encoder_unit = (2 * 3.14 * wheel_radius) / 6.28;
    
   
    // Main loop
    while (wb_robot_step(TIME_STEP) != -1) {
        debug_log("Running...");
        generateInitialWalls();
        
        API_moveForward();

        for (int x = 0; x < 20; x+= 2 ) {
            for (int y = 0; y < 20; y++) {
                if (travelArray[x][y] != 1) {
                    num = 0;
                    LOWER_X_GOAL = x;
                    LOWER_Y_GOAL = y;
                    UPPER_X_GOAL = x;
                    UPPER_Y_GOAL = y;
                    while (num < 1) {
                        Action nextMove = solver();
                    }
                }
            }
            for (int y = 19; y >= 0; y --) {  // Decreasing step of 2
                if (travelArray[x][y] != 1) {
                    num = 0;
                    LOWER_X_GOAL = x+1;
                    LOWER_Y_GOAL = y;
                    UPPER_X_GOAL = x+1;
                    UPPER_Y_GOAL = y;
                    while (num < 1) {
                        Action nextMove = solver();
                        num++;  // Ensure loop exits
                    }
                }
            }
        }

        num = 0;
        LOWER_X_GOAL = 10;
        LOWER_Y_GOAL = 0;
        UPPER_X_GOAL = 10;
        UPPER_Y_GOAL = 0;
        while (num < 1) {
            Action nextMove = solver();
        }

        debug_log("Dry Run Complete");
        debug_log("Returned to starting position");
        debug_log("Identified survivors");
        debug_log("Saving survivors");
        
        for (int x = 0; x < 20; x++) {
            for (int y = 0; y < 20; y++) {
                if (survivorArray[x][y] == 1) {
                    num = 0;
                    LOWER_X_GOAL = x;
                    LOWER_Y_GOAL = y;
                    UPPER_X_GOAL = x;
                    UPPER_Y_GOAL = y;
                    while (num < 1) {
                        Action nextMove = solver();
                    }
                    debug_log("Entered the tile with the survivor");
                    double start_time = wb_robot_get_time();  // Get current simulation time
                    while (wb_robot_get_time() - start_time < 3.0) {
                        wb_robot_step(TIME_STEP);  // Keep stepping until 3 seconds pass
                    }
 
                }
 
            }
        }

        num = 0;
        LOWER_X_GOAL = 10;
        LOWER_Y_GOAL = 0;
        UPPER_X_GOAL = 10;
        UPPER_Y_GOAL = 0;
        while (num < 1) {
            Action nextMove = solver();
        }
        API_turnRight();
        API_moveForward();
        
        
    debug_log("Task Complete");  
    break;
    
    }
    

    wb_robot_cleanup();
    return 0;
    
    
}