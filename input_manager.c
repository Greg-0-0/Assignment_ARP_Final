// gcc input_manager.c -lncurses -o input_manager
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#include"functions.h"

int main(int argc, char* argv[]){
    if(argc < 5){
        fprintf(stderr,"No arguments passed to input manager\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0);// Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("INPUT_MANAGER sem_open");
        exit(EXIT_FAILURE);
    }

    // Process identification logging
    pid_t pid = getpid();
    write_process_pid("processes.log", "INPUT_MANAGER", pid, log_sem);

    // Process started successfully
    write_log("application.log", "INPUT_MANAGER", "INFO", "Input Manager process started successfully", log_sem);

    int fd_key = atoi(argv[1]); // File descriptor to write key inputs to drone 
    int fd_bb = atoi(argv[2]); // File descriptor to read score and drone position from blackboard
    int fd_hb_watchdog = atoi(argv[3]); // Heartbeat pipe to watchdog
    int application_flag = atoi(argv[4]); // Flag indicating standalone or networked mode

    // Make read from fd_bb non-blocking, this way inputs can be read continuously
    int flags = fcntl(fd_bb, F_GETFL, 0);
    if(flags < 0){
        log_error("application.log", "INPUT_MANAGER", "fcntl F_GETFL", log_sem);
        perror("INPUT_MANAGER line-33 fcntl F_GETFL");
        exit(EXIT_FAILURE);
    }
    flags |= O_NONBLOCK;
    if(fcntl(fd_bb,F_SETFL, flags) < 0){
        log_error("application.log", "INPUT_MANAGER", "fcntl F_SETFL", log_sem);
        perror("INPUT_MANAGER line-39 fcntl F_SETFL");
        exit(EXIT_FAILURE);
    }

    int y, x, drone_x = 0, drone_y = 0, score = 0, obstacle_interaction = 0;
    BlackboardMsg positions; positions.type = MSG_NAN;

    // Use int: ncurses special keys (e.g., KEY_RESIZE) are defined as int, this avoids implict casting when comparing
    int input_key = ERR;
    initscr();
    start_color();
    resize_term(0, 0);
    clear();
    cbreak();
    noecho();
    keypad(stdscr, TRUE); // To receive KEY_RESIZE
    nodelay(stdscr, TRUE); // getch() becomes non-blocking, this way there is no wait for resizing, and code can keep executing

    getmaxyx(stdscr, y, x);
    drone_x = x/2;
    drone_y = y/2;
    mvprintw(y/2, x/2, "d");
    mvprintw(y/2, x/2+1, "f");
    mvprintw(y/2-1, x/2+1, "r");
    mvprintw(y/2+1, x/2+1, "v");
    mvprintw(y/2+1, x/2-1, "x");
    mvprintw(y/2-1, x/2-1, "w");
    mvprintw(y/2, x/2-1, "s");
    mvprintw(y/2-1, x/2, "e");
    mvprintw(y/2+1, x/2, "c");
    mvprintw(y/2+7, x/2-7, "PRESS q TO EXIT");
    if(application_flag)
        mvprintw(y/2-7, x/2-15, "Score: 0");
    else
        mvprintw(y/2-7, x/2-15, "Score: -");
    mvprintw(y/2-7, x/2+8, "X drone pos: %d", drone_x);
    mvprintw(y/2-6, x/2+8, "Y drone pos: %d", drone_y);
    refresh();

    if(application_flag)
        // Heartbeat using SIGALRM + ITIMER (sigaction with SA_RESTART inside)
        setup_heartbeat_itimer(1);

    while(1){
        
        // Read updated drone position and score from blackboard
        int bytes_read = read(fd_bb, &positions, sizeof(positions));
        if (bytes_read > 0) {
            if(!application_flag && positions.type == MSG_QUIT){
                // Quitting program in networked mode (for client if terminated by user from server, or for server if terminated after disconnection)

                // Notify drone about termination as well
                char key_to_send = 'q';
                write(fd_key, &key_to_send,1);
                write_log("application.log", "INPUT_MANAGER", "INFO", "Input Manager process terminated successfully", log_sem);
                exit(EXIT_SUCCESS);
            }
            else if(positions.type == MSG_NPOS){

                // Update display and local variables with new position and score
                drone_x = positions.drone_x;
                drone_y = positions.drone_y;
                if(application_flag){
                    score = positions.reached_targets;
                    mvprintw(y/2-7, x/2-15, "Score: %d", positions.reached_targets);
                }
                mvprintw(y/2-7, x/2+8, "X drone pos: %d", positions.drone_x);
                mvprintw(y/2-6, x/2+8, "Y drone pos: %d", positions.drone_y);
                refresh();
            }
            else if(!application_flag && positions.type == MSG_NOB)
                // Update drone position after obstacle sent from client moved (networked mode) (only for server)
                obstacle_interaction = 1;
        }
        
        if(application_flag)
            // Heartbeat if due
            send_heartbeat_if_due(fd_hb_watchdog, "INPUT_MANAGER", log_sem);
        
        input_key = getch();

        int new_y, new_x;
        getmaxyx(stdscr, new_y, new_x);

        if (input_key == KEY_RESIZE || is_term_resized(y, x)) {
            // Storing new values
            y = new_y;
            x = new_x;

            input_key = ERR; // Reset input key to avoid interfering with other inputs

            // Resize window
            resize_term(0, 0);
            start_color();
            werase(stdscr);

            getmaxyx(stdscr,y, x);
            mvprintw(y/2, x/2, "d");
            mvprintw(y/2, x/2+1, "f");
            mvprintw(y/2-1, x/2+1, "r");
            mvprintw(y/2+1, x/2+1, "v");
            mvprintw(y/2+1, x/2-1, "x");
            mvprintw(y/2-1, x/2-1, "w");
            mvprintw(y/2, x/2-1, "s");
            mvprintw(y/2-1, x/2, "e");
            mvprintw(y/2+1, x/2, "c");
            mvprintw(y/2+7, x/2-7, "PRESS q TO EXIT");
            if(application_flag)
                mvprintw(y/2-7, x/2-15, "Score: %d", score);
            else
                mvprintw(y/2-7, x/2-15, "Score: -");
            mvprintw(y/2-7, x/2+8, "X drone pos: %d", drone_x);
            mvprintw(y/2-6, x/2+8, "Y drone pos: %d", drone_y);
            refresh();
        }
        if(input_key == 'w' || input_key == 'e' || input_key == 'r' || input_key == 's'|| input_key == 'd' || 
            input_key == 'f' || input_key == 'x' || input_key == 'c' || input_key == 'v' || input_key == 'q' || 
            obstacle_interaction == 1){
                // Send one byte to the drone
                char key_to_send = (char)input_key;
                if(obstacle_interaction == 1){
                    // Artifical key sent to the drone, since drone didn't move but obstacle did
                    key_to_send = 'o'; // Indicate obstacle position update to drone
                    input_key = key_to_send; // For logging purpose
                    obstacle_interaction = 0; // Reset flag
                }
                write(fd_key, &key_to_send, 1);
                if(input_key == 'q'){
                    write_log("application.log", "INPUT_MANAGER", "INFO", "Input Manager process terminated successfully", log_sem);
                    exit(EXIT_SUCCESS);
                }
                char log_msg[50];
                snprintf(log_msg, sizeof log_msg, "Key pressed: %c", input_key);
                write_log("application.log", "INPUT_MANAGER", "INFO", log_msg, log_sem);

                input_key = ERR; // Reset input key to avoid interfering with other inputs
        }
        
        if(application_flag)
            // Heartbeat at end of loop
            send_heartbeat_if_due(fd_hb_watchdog, "INPUT_MANAGER", log_sem);
        
        // Small delay to avoid busy-waiting
        //sleep_ms(10); // 10ms
    }   

    close(fd_key);
    close(fd_bb);

    sem_close(log_sem);

    getch();
    endwin();
    return 0;
}