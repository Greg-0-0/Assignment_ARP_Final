#define _POSIX_C_SOURCE 200809L

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
#include <semaphore.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include<netdb.h>
#include <netinet/tcp.h>

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
    MSG_NTARGET = 6,
    MSG_WSIZE = 7
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
        int reached_targets;
        int obstacles[N_OBS][2];
        int targets[N_TARGETS][2];
} BlackboardMsg;

// Global variable to signal obstacle position update
extern volatile sig_atomic_t update_obstacles;
extern volatile sig_atomic_t heartbeat_due;

// ------ used in blackboard.c ------

// Function to draw a red rectangle on the border
void draw_rect(WINDOW *win, int y, int x, int h, int w, int color_pair);

// Function to create the outer window where the drone moves
void layout_and_draw(WINDOW *win);

// Function to create the outer window for networked application
void layout_and_draw_for_networked_app(WINDOW *win, int server_client_flag, int height, int width);

// Sets flag to change obstacle position
void change_obstacle_position_flag();

// Function to check how many targets have been reached and request new ones if all reached
void check_targets_reached(BlackboardMsg* positions, WINDOW* win, 
    int* reached_targets, int fd_trs, int fd_npos_to_t, sem_t *log_sem);

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
    double loop_next_drone_pos[2], int previous_drone_pos[2], int next_drone_pos[2], double* temp_x, double* temp_y, sem_t *log_sem);

// Lightweight repulsion update for networked mode (single step, no delay)
// Returns 1 if quit was requested, 0 otherwise
int apply_single_repulsion_step(int fd_npos, int fd_key, DroneMsg* drone_msg, double max_force, double M, double K, double T, 
    int obstacles[N_OBS][2], int ro, int borders[2], sem_t *log_sem);

// Function to implement drone movement and border repulsion
// Returns 1 if user pressed 'q' to quit, 0 otherwise
int move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y, double max_force, 
       double oblique_force_comp, double M, double K, double T, int borders[], int obstacles[N_OBS][2], int ro, sem_t *log_sem);

// ------ used in obstacles.c & targets.c ------

// Function to check whether the new obstacle/target position is valid
int check_position(int new_y, int new_x, BlackboardMsg positions, int n_spawned_elem,
     int obstacles_spawned, int targets_spawned);

// Function used to ensure all the bytes composing the message received are read
ssize_t read_full(int fd, void* buf, size_t size);

// ------ used in master.c ------

// Function to run separte processes
int spawn(const char *prog, char *const argv[]);

// ------ used in master.c & input_manager.c & blackboard.c & obstacles.c & targets.c ------

// Function to identify process pid and write it to process log file
void write_process_pid(const char* log_filename, const char* process_name, pid_t pid, sem_t *log_sem);

// Function to write log messages to a log file
void write_log(const char* log_filename, const char* process_name,
     const char* level, const char* message, sem_t *log_sem);

// Function to write error messages to a log file
void log_error(const char* log_filename, const char* process_name,
     const char* context, sem_t *log_sem);

// ------ heartbeat helpers (used by all processes) ------

// Per-process heartbeat flag set by a timer signal
extern volatile sig_atomic_t heartbeat_due;

// Generic signal handler to set the heartbeat flag
void heartbeat_signal_handler(int signo);

// Use ITIMER_REAL + SIGALRM for a 1-second heartbeat (processes not using SIGALRM)
void setup_heartbeat_itimer(int interval_sec);

// Use POSIX timer with a custom signal (e.g., SIGRTMIN) for blackboard
int setup_heartbeat_posix_timer(int interval_sec, int signo);

// If a heartbeat is due, send it via fd to watchdog (writes pid)
void send_heartbeat_if_due(int fd_watchdog, const char* process_name, sem_t *log_sem);

// ------ socket_manager helpers (used by socket_manager.c) ------

// Function to analyze dron position/window size and prepare fixed message to send to client
void analyze_position_n_size_and_prepare_message(BlackboardMsg positions, char* buffer_output, int wind_H);

// Helper function to print error message, close sockets and exit
void error(int newsockfd, int sockfd, const char *msg, sem_t *log_sem);

// Helper function to read a line (ending with '\n') from the socket, this avoids mixing messages
ssize_t read_line(int fd, char *buf, size_t maxlen);

#endif