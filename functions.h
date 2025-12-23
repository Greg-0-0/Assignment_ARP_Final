#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<ncurses.h>
#include<locale.h>
#include<math.h>
#include<time.h>
#include<sys/select.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>

#define N_OBS 10
#define N_TARGETS 9

// Enum variable to classify every type of message sent via pipe
typedef enum{
    MSG_QUIT = 0,
    MSG_POS = 1,
    MSG_NPOS = 2,
    MSG_STOP = 3,
    MSG_NAN = 4,
    MSG_NOB = 5,
    MSG_NTARGET = 6
} MsgType;

// Struct to define message sent by drone to blackboard with new drone position
typedef struct{
    MsgType type;
    int new_drone_y;
    int new_drone_x;
} DroneMsg;

// Struct used by blackboard to communicate to drone program the border, obstacles and current drone position
typedef struct{
        MsgType type;
        int drone_y;
        int drone_x;
        int border_y;
        int border_x;
        int obstacles[N_OBS][2];
        int targets[N_TARGETS][2];
} BlackboardMsg;

// Global variable to signal obstacle position update
extern volatile sig_atomic_t update_obstacles;

// ------ used in blackboard.c ------

// Function to draw a red rectangle on the border
void draw_rect(WINDOW *win, int y, int x, int h, int w, int color_pair);

// Function to create the outer window where the drone moves
void layout_and_draw(WINDOW *win);

// Sets flag to change obstacle position
void change_obstacle_position_flag();

// ------ used in drone.c ------

// Scanning a string to remove its white spaces
char* remove_white_space(char* string);

// Copying parameter values on local variables
int load_parameters(const char* filename, double* drone_mass, double* air_resistance, double* integr_inter, int* rep_radius, double* max_force);

// Custom function to make the process sleep for ms milliseconds
void sleep_ms(long ms);

// Function used to safely empty a pipe
void drain_pipe(int fd);

// Function to compute repulsive forces of obstacles
void compute_repulsive_forces(int fd_npos,DroneMsg* drone_msg, double* force_x, double* force_y, double max_force,
    double M, double K, double T, int obstacles[N_OBS][2], int ro, double loop_prev_drone_pos[2],double loop_curr_drone_pos[2],
    double loop_next_drone_pos[2], int previous_drone_pos[2], int next_drone_pos[2], double* temp_x, double* temp_y);

// Function to implement drone movement and border repulsion
void move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y, double max_force, 
       double oblique_force_comp, double M, double K, double T, int borders[], int obstacles[N_OBS][2], int ro);

// ------ used in obstacles.c ------

// Function to check wether new obstacle position is valid
int check_position(int new_y, int new_x, BlackboardMsg positions);

// ------ used in targets.c ------

void check_targets_reached(BlackboardMsg* positions, WINDOW* win, int* reached_targets, int fd_trs, int fd_npos_to_t);

// ------ used in obstacles.c & targets.c ------

// Function used to ensure all the bytes composing the message received are read
ssize_t read_full(int fd, void* buf, size_t size);

// ------ used in master.c ------

// Function to run separte processes
void spawn(const char *prog, char *const argv[]);

#endif