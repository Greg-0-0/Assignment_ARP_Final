// gcc -Wall -Wextra blackboard.c -lncurses -o blackboard
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#define _XOPEN_SOURCE_EXTENDED
#include"functions.h"

int main(int argc, char* argv[]) {

    if(argc < 8){
        fprintf(stderr,"No arguments passed to blackboard\n");
        exit(EXIT_FAILURE);
    }

    int fd_req = atoi(argv[1]); // Receives request of new position from drone, non blocking otherwise it would block resize of window
    int fd_npos = atoi(argv[2]); // Receives new position of drone from drone (movement), can be blocking, since we are waiting for a new position, 
    // after pressing a key, and realistically we wouldn't choose to resize window at that moment
    int fd_pos = atoi(argv[3]); // Used to send current drone position, obstacles position and borders position to drone
    int fd_npos_to_o = atoi(argv[4]); // Writes new drone and borders position to obstacle program after resizing
    int fd_nobs = atoi(argv[5]); // Reads new obstacles position
    int fd_npos_to_t = atoi(argv[6]); // Writes new drone, borders and obstacles position after resizing
    int fd_trs = atoi(argv[7]); // Reads new targets position

    int H, W, reached_targets = 0;

    // Make reads from fd_req and fd_npos not blocking
    int flags = fcntl(fd_req, F_GETFL, 0);
    if(flags < 0){
        perror("fcnt F_GETFl");
        exit(EXIT_FAILURE);
    }
    flags |= O_NONBLOCK;
    if(fcntl(fd_req,F_SETFL, flags) < 0){
        perror("fnctl F_SETFL");
    }

    flags = fcntl(fd_npos, F_GETFL, 0);
    if(flags < 0){
        perror("fcnt F_GETFl");
        exit(EXIT_FAILURE);
    }
    flags |= O_NONBLOCK;
    if(fcntl(fd_npos,F_SETFL, flags) < 0){
        perror("fnctl F_SETFL");
    }

    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0;
           positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_POS;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    // Timer setup for obstacle position change
    static struct itimerval timer;
    timer.it_interval.tv_sec = 5;
    timer.it_interval.tv_usec = 0;
    timer.it_value.tv_sec = 5;
    timer.it_value.tv_usec = 0;


    setlocale(LC_ALL, "");

    initscr();
    start_color(); // For window synchronization
    resize_term(0, 0);
    clear();
    refresh();
    cbreak();
    noecho();
    keypad(stdscr, TRUE); // To receive KEY_RESIZE
    nodelay(stdscr, TRUE); // getch() becomes non-blocking, this way there is no wait for resizing, and code can keep executing

    // Window with temporary dimensions
    WINDOW *win = newwin(3, 3, 0, 0);
    layout_and_draw(win);
    getmaxyx(win, H, W);

    // Initial drone position
    positions.drone_y = H/2; // y
    positions.drone_x = W/2; // x

    // Saving borders value
    positions.border_y = H-7;
    positions.border_x = W-7;

    refresh();
    wrefresh(win);
    
    // Retrieving obstacles position
    write(fd_npos_to_o,&positions,sizeof(positions));
    read(fd_nobs,&positions,sizeof(positions));

    // Printing obstacles
    for(int i = 0;i<N_OBS;i++){
        wattron(win, COLOR_PAIR(3));
        mvwprintw(win,positions.obstacles[i][0],positions.obstacles[i][1],"o");
        wattroff(win, COLOR_PAIR(3));
        wrefresh(win);
    }

    // Retrieving targets position
    write(fd_npos_to_t,&positions,sizeof(positions));
    read(fd_trs,&positions,sizeof(positions));

    // Printing targets
    for(int i = 0;i<N_TARGETS;i++){
        wattron(win, COLOR_PAIR(2));
        mvwprintw(win,positions.targets[i][0],positions.targets[i][1],"%d",i+1);
        wattroff(win, COLOR_PAIR(2));
        wrefresh(win);
    }

    // Set up signal handler to change obstacle position periodically
    signal(SIGALRM, change_obstacle_position_flag);

    // Starting timer for obstacle position change every 5 seconds
    setitimer(ITIMER_REAL, &timer, NULL);

    sleep(5);
    
    while (1) {
        if (getch() == KEY_RESIZE) {

            // Resize window
            resize_term(0, 0);
            layout_and_draw(win);
            getmaxyx(win, H, W);

            // Reset drone position
            positions.drone_y = H/2; // y
            positions.drone_x = W/2; // x

            // Updating borders (margins of 5 pixels)
            positions.border_y = H-7;
            positions.border_x = W-7;

            int temp = positions.type;
            positions.type = MSG_POS;

            // Retrieving obstacles position 
            write(fd_npos_to_o,&positions,sizeof(positions));
            read(fd_nobs,&positions,sizeof(positions));

            // Printing obstacles
            for(int i = 0;i<N_OBS;i++){
                wattron(win, COLOR_PAIR(3));
                mvwprintw(win,positions.obstacles[i][0],positions.obstacles[i][1],"o");
                wattroff(win, COLOR_PAIR(3));
                wrefresh(win);
            }

            // Retrieving targets position
            write(fd_npos_to_t,&positions,sizeof(positions));
            read(fd_trs,&positions,sizeof(positions));

            // Printing targets
            for(int i = 0;i<N_TARGETS;i++){
                wattron(win, COLOR_PAIR(2));
                mvwprintw(win,positions.targets[i][0],positions.targets[i][1],"%d",i+1);
                wattroff(win, COLOR_PAIR(2));
                wrefresh(win);
            }

            positions.type = temp;

        }
        if(update_obstacles){

            // Time to change obstacle position
            update_obstacles = 0;

            // Choose an obstacle to move
            srand(time(NULL));
            int obs_to_move = rand() % N_OBS;
            mvwaddch(win,positions.obstacles[obs_to_move][0],positions.obstacles[obs_to_move][1],' '); // Cleaning previous obstacle position
            // Telling obstacle program which obstacle to move
            positions.obstacles[obs_to_move][0] = -1;
            positions.obstacles[obs_to_move][1] = -1;
            int temp = positions.type;
            positions.type = MSG_NOB;
            // Retrieving new obstacle position
            write(fd_npos_to_o,&positions,sizeof(positions));
            read(fd_nobs,&positions,sizeof(positions));
            // Printing new obstacle
            wattron(win, COLOR_PAIR(3));
            mvwprintw(win,positions.obstacles[obs_to_move][0],positions.obstacles[obs_to_move][1],"o");
            wattroff(win, COLOR_PAIR(3));
            wrefresh(win);
            
            positions.type = temp;
        }
        read(fd_req, &drone_msg, sizeof(drone_msg)); // Checks for position request from drone (non blocking)
        if(drone_msg.type == MSG_POS){
            // Drone is asking for position
            write(fd_pos,&positions,sizeof(positions)); // Sends current position
            // If drone is asking for position it wants to move, thus reading on fd_npos
            while(1){
                read(fd_npos,&drone_msg,sizeof(drone_msg)); // Receiving new position
                if(drone_msg.type == MSG_STOP || drone_msg.type == MSG_NAN){
                    break;
                }
                if(drone_msg.type == MSG_QUIT){
                    // Quitting program
                    positions.type = MSG_QUIT;
                    write(fd_npos_to_o,&positions,sizeof(positions)); // Sendds message to obstacles program
                    write(fd_npos_to_t,&positions,sizeof(positions)); // Sendds message to targets program
                    exit(EXIT_SUCCESS);
                }
                mvwaddch(win,positions.drone_y,positions.drone_x,' '); // Cleaning previous drone position
                // Resetting eventual drone position off the window 
                // (this can happen even though the drone is then pushed back inside the border by the force of the fence)
                if(drone_msg.new_drone_y >= H-2)
                    drone_msg.new_drone_y= H-2;
                else if(drone_msg.new_drone_y < 1)
                    drone_msg.new_drone_y = 1;
                if(drone_msg.new_drone_x >= W-2)
                    drone_msg.new_drone_x = W-2;
                else if(drone_msg.new_drone_x < 1)
                    drone_msg.new_drone_x = 1;
                positions.drone_y = drone_msg.new_drone_y;
                positions.drone_x = drone_msg.new_drone_x;
                wattron(win, COLOR_PAIR(4));
                mvwprintw(win,positions.drone_y,positions.drone_x,"+"); // Drawing drone at new position
                wattroff(win, COLOR_PAIR(4));
                draw_rect(win,6,6,H-7,W-7,1);
                check_targets_reached(&positions, win, &reached_targets, fd_trs, fd_npos_to_t);
                wrefresh(win);
            }
        }
        else if(drone_msg.type == MSG_QUIT){
            positions.type = MSG_QUIT;
            write(fd_npos_to_o,&positions,sizeof(positions));
            write(fd_npos_to_t,&positions,sizeof(positions));
            exit(EXIT_SUCCESS);
        }
        wrefresh(win);
    }

    close(fd_req);
    close(fd_npos);
    close(fd_pos);
    close(fd_npos_to_o);
    close(fd_nobs);
    close(fd_npos_to_t);
    close(fd_trs);

    delwin(win);
    endwin();
    return 0;
}