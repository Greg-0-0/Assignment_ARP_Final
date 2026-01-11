#define _XOPEN_SOURCE_EXTENDED
#include"functions.h"

int main(int argc, char* argv[]) {

    if(argc < 9){
        fprintf(stderr,"No arguments passed to blackboard server\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0); // Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("sem_open");
        exit(EXIT_FAILURE);
    }

    // Avoid process termination on writes to closed pipes (e.g., possibly input_manager after exit -> line 280)
    // Allows graceful handling of EPIPE errors instead of abrupt termination
    signal(SIGPIPE, SIG_IGN);

    // Process started successfully
    write_log("application.log", "BLACKBOARD", "INFO", "Blackboard process started successfully", log_sem);

    int fd_req = atoi(argv[1]); // Receives request of new position from drone, non blocking otherwise it would block resize of window
    int fd_npos = atoi(argv[2]); // Receives new position of drone from drone (movement), can be blocking, since we are waiting for a new position, 
    // after pressing a key, and realistically we wouldn't choose to resize window at that moment
    int fd_pos = atoi(argv[3]); // Used to send current drone position, obstacle position and borders position to drone
    int fd_ninfo_to_im = atoi(argv[4]); // Writes new drone position to input manager for visual update
    int fd_to_sm = atoi(argv[5]); // Writes drone position (seen as obstacle from other application -> server) 
    // (or window size -> not sure yet) to socket manager (sent to other application)
    int fd_from_sm_1 = atoi(argv[6]); // Reads window size from socket manager (retrieved from other application, blocking)
    int fd_from_sm_2 = atoi(argv[7]); // Reads drone position from socket manager (retrieved from other application, non-blocking)
    int fd_to_sm_2 = atoi(argv[8]); // Writes quit message to socket manager to exit it correctly

    int H, W;
    int server_drone_pos[2]; server_drone_pos[0] = 0; server_drone_pos[1] = 0;

    // Make read from fd_req non-blocking; keep fd_npos blocking to avoid stale reads
    int flags = fcntl(fd_req, F_GETFL, 0);
    if(flags < 0){
        perror("fcnt F_GETFl");
        exit(EXIT_FAILURE);
    }
    flags |= O_NONBLOCK;
    if(fcntl(fd_req,F_SETFL, flags) < 0){
        perror("fnctl F_SETFL");
    }

    // Guarantee blocking read on fd_npos (ensure correct MSG_TYPE reception in inner while loop)
    int npos_flags = fcntl(fd_npos, F_GETFL, 0);
    if(npos_flags < 0){
        perror("fcnt F_GETFl");
        exit(EXIT_FAILURE);
    }
    npos_flags &= ~O_NONBLOCK;
    if(fcntl(fd_npos,F_SETFL, npos_flags) < 0){
        perror("fnctl F_SETFL");
    }

    // Make write to input manager non-blocking to avoid blocking when it exits
    int im_flags = fcntl(fd_ninfo_to_im, F_GETFL, 0);
    if(im_flags >= 0){
        im_flags |= O_NONBLOCK;
        (void)fcntl(fd_ninfo_to_im, F_SETFL, im_flags);
    }

    // Make write to socket manager non-blocking
    int sm_flags = fcntl(fd_to_sm, F_GETFL, 0);
    if(sm_flags >= 0){
        sm_flags |= O_NONBLOCK;
        (void)fcntl(fd_to_sm, F_SETFL, sm_flags);
    }
    else{
        perror("fcntl F_GETFL for socket manager");
    }

    // Make second read from socket manager non-blocking
    int from_sm_2_flags = fcntl(fd_from_sm_2, F_GETFL, 0);
    if(from_sm_2_flags >= 0){
        from_sm_2_flags |= O_NONBLOCK;
        (void)fcntl(fd_from_sm_2, F_SETFL, from_sm_2_flags);
    }
    else{
        perror("fcntl F_GETFL for socket manager");
    }

    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0; positions.reached_targets = 0;
           positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_POS;
    // Separate variable for socket manager reads
    BlackboardMsg positions_sm; positions_sm.border_x = 0; positions_sm.border_y = 0; positions_sm.reached_targets = 0;
           positions_sm.drone_x = 0; positions_sm.drone_y = 0; positions_sm.type = MSG_NAN;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    // Initializing obstacles positions to -1 (not present in client)
    for(int i = 0;i<N_OBS;i++){
        positions.obstacles[i][0] = -1;
        positions.obstacles[i][1] = -1;
    }

    setlocale(LC_ALL, "");

    initscr();
    start_color(); // For window synchronization
    resize_term(0, 0);
    clear();
    refresh();
    cbreak();
    noecho();
    //keypad(stdscr, TRUE); // To receive KEY_RESIZE
    nodelay(stdscr, TRUE); // getch() becomes non-blocking, this way there is no wait for resizing, and code can keep executing

    // Retrieve window size from socket manager (blocking)
    read(fd_from_sm_1, &positions_sm, sizeof(positions_sm));
    if(positions_sm.type != MSG_WSIZE){
        // Error in receiving window size
        write_log("application.log", "BLACKBOARD", "ERROR", "Error receiving window size from socket manager", log_sem);
        exit(EXIT_FAILURE);
    }   

    // Window with temporary dimensions
    WINDOW *win = newwin(3, 3, 0, 0);
    layout_and_draw_for_networked_app(win, 1, positions_sm.border_y, positions_sm.border_x); // Client mode
    getmaxyx(win, H, W);

    // Initial drone position
    positions.drone_y = H/2; // y
    positions.drone_x = W/2 + 7; // x

    // Saving borders value
    positions.border_y = H-7;
    positions.border_x = W-7;

    // Initial drone position of server application
    positions_sm.drone_y = H/2; // y
    positions_sm.drone_x = W/2 - 7; // x

    // Updating local variable for server drone position
    server_drone_pos[0] = positions_sm.drone_y;
    server_drone_pos[1] = positions_sm.drone_x;

    refresh();
    wrefresh(win);
    
    while (1) {
        // Receiving request from socket manager about quit, obstacle (client drone), or new drone position (non-blocking)
        read(fd_from_sm_2, &positions_sm, sizeof(positions_sm));
        if(positions_sm.type == MSG_NPOS){
            // Displaying new drone position of server application (position automatically saved in positions_sm)

            positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

            if(server_drone_pos[0] != positions_sm.drone_y || server_drone_pos[1] != positions_sm.drone_x){
                // Cleaning previous position only if it has changed, this avoids continuous updates
                mvwaddch(win,server_drone_pos[0],server_drone_pos[1],' ');
                server_drone_pos[0] = positions_sm.drone_y;
                server_drone_pos[1] = positions_sm.drone_x;
                wattron(win, COLOR_PAIR(3));
                mvwaddch(win,positions_sm.drone_y,positions_sm.drone_x,'o'); // Drawing server drone at new position
                wattroff(win, COLOR_PAIR(3));
                mvwprintw(win,H/2 + 7,W/2,"Server Drone Pos: (%d, %d) ", positions_sm.drone_y, positions_sm.drone_x);
                wrefresh(win);
            }
        }
        else if(positions_sm.type == MSG_NOB){
            // Request drone position seen as obstacle from other application

            positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

            write(fd_to_sm, &positions, sizeof(positions));
        }
        else if(positions_sm.type == MSG_QUIT){
            // Quitting program from server application

            positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

            // Must notify input manager about termination as well 
            // -> input manager notifies drone which notifies blackboardclient
            int temp = positions.type;
            positions.type = MSG_QUIT;
            ssize_t nw = write(fd_ninfo_to_im, &positions, sizeof(positions));
            (void)nw; // Non-blocking: intentionally ignore if input_manager has already exited
            positions.type = temp;
        }

        read(fd_req, &drone_msg, sizeof(drone_msg)); // Checks for position request from drone (non blocking)
        if(drone_msg.type == MSG_POS){
            // Drone is asking for position
            write(fd_pos,&positions,sizeof(positions)); // Sends current position
            // If drone is asking for position it wants to move, thus reading on fd_npos
            while(1){
                
                ssize_t nread = read(fd_npos,&drone_msg,sizeof(drone_msg)); // Receiving new position (blocking)
                if(nread != (ssize_t)sizeof(drone_msg)){
                    // Ignore interrupts/partial reads (in case drone sent incomplete message)
                    continue;
                }
                if(drone_msg.type == MSG_STOP || drone_msg.type == MSG_NAN){
                    break;
                }
                if(drone_msg.type == MSG_QUIT){
                    // Quitting program

                    // Send termination message to socket manager to close socket and exit correctly 
                    positions.type = MSG_QUIT;
                    write(fd_to_sm_2, &positions, sizeof(positions));

                    write_log("application.log", "BLACKBOARD", "INFO", "Blackboard process terminated successfully", log_sem);
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
                wrefresh(win);

                // Sending new position and score to input manager for visual update
                ssize_t nw = write(fd_ninfo_to_im, &positions, sizeof(positions));
                (void)nw; // Non-blocking: intentionally ignore if input_manager has already exited

                // Receiving request from socket manager about quit, new drone position, or new obstacle (non-blocking)
                read(fd_from_sm_2, &positions_sm, sizeof(positions_sm));
                if(positions_sm.type == MSG_NPOS){
                    // Displaying new drone position received from other application

                    positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

                    if(server_drone_pos[0] != positions_sm.drone_y && server_drone_pos[1] != positions_sm.drone_x){
                        // Cleaning previous position only if it has changed, this avoids continuous updates
                        mvwaddch(win,server_drone_pos[0],server_drone_pos[1],' ');
                        server_drone_pos[0] = positions_sm.drone_y;
                        server_drone_pos[1] = positions_sm.drone_x;
                        wattron(win, COLOR_PAIR(3));
                        mvwaddch(win,positions_sm.drone_y,positions_sm.drone_x,'o'); // Drawing server drone at new position
                        wattroff(win, COLOR_PAIR(3));
                        mvwprintw(win,H/2 + 7,W/2,"Server Drone Pos: (%d, %d) ", positions_sm.drone_y, positions_sm.drone_x);
                        wrefresh(win);
                    }
                }
                else if(positions_sm.type == MSG_NOB){
                    // Request drone position seen as obstacle from other application

                    positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

                    write(fd_to_sm, &positions, sizeof(positions));
                }
                else if(positions_sm.type == MSG_QUIT){
                    // Quitting program from server application

                    positions_sm.type = MSG_NAN; // Resetting type to avoid stale reads

                    // Must notify input manager about termination as well 
                    // -> input manager notifies drone which notifies blackboardclient
                    int temp = positions.type;
                    positions.type = MSG_QUIT;
                    ssize_t nw = write(fd_ninfo_to_im, &positions, sizeof(positions));
                    (void)nw; // Non-blocking: intentionally ignore if input_manager has already exited
                    positions.type = temp;
                }
            }
        }
        else if(drone_msg.type == MSG_QUIT){
            // Quitting program

            // Send termination message to socket manager to close socket and exit correctly 
            positions.type = MSG_QUIT;
            write(fd_to_sm_2, &positions, sizeof(positions));

            write_log("application.log", "BLACKBOARD", "INFO", "Blackboard process terminated successfully", log_sem);
            exit(EXIT_SUCCESS);
        }

        wrefresh(win);
    }

    close(fd_req);
    close(fd_npos);
    close(fd_pos);
    close(fd_to_sm);
    close(fd_from_sm_1);
    close(fd_from_sm_2);
    close(fd_ninfo_to_im);

    sem_close(log_sem);
    sem_unlink("/log_sem");

    delwin(win);
    endwin();
    return 0;
}