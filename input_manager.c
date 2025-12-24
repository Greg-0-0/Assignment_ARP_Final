// gcc input_manager.c -lncurses -o input_manager
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#include"functions.h"

int main(int argc, char* argv[]){
    if(argc < 3){
        fprintf(stderr,"No arguments passed to input manager\n");
        exit(EXIT_FAILURE);
    }

    int fd_key = atoi(argv[1]); // File descriptor to write key inputs to drone 
    int fd_bb = atoi(argv[2]); // File descriptor to read score and drone position from blackboard

    // Make read from fd_bb non-blocking, this way inputs can be read continuously
    int flags = fcntl(fd_bb, F_GETFL, 0);
    if(flags < 0){
        perror("fcnt F_GETFl");
        exit(EXIT_FAILURE);
    }
    flags |= O_NONBLOCK;
    if(fcntl(fd_bb,F_SETFL, flags) < 0){
        perror("fnctl F_SETFL");
    }

    int y, x, drone_x = 0, drone_y = 0, score = 0;
    BlackboardMsg positions; positions.type = MSG_NAN;

    char input_key = 'o';
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
    mvprintw(y/2-7, x/2-15, "Score: 0");
    mvprintw(y/2-7, x/2+8, "X drone pos: %d", drone_x);
    mvprintw(y/2-6, x/2+8, "Y drone pos: %d", drone_y);
    refresh();

    while(1){
        
        // Read updated drone position and score from blackboard
        int bytes_read = read(fd_bb, &positions, sizeof(positions));
        if (bytes_read > 0) {

            // Update display and local variables with new position and score
            drone_x = positions.drone_x;
            drone_y = positions.drone_y;
            score = positions.reached_targets;
            mvprintw(y/2-7, x/2-15, "Score: %d", positions.reached_targets);
            mvprintw(y/2-7, x/2+8, "X drone pos: %d", positions.drone_x);
            mvprintw(y/2-6, x/2+8, "Y drone pos: %d", positions.drone_y);
            refresh();
        }
        
        
        input_key = getch();

        int new_y, new_x;
        getmaxyx(stdscr, new_y, new_x);

        if (input_key == KEY_RESIZE || is_term_resized(y, x)) {
            // Storing new values
            y = new_y;
            x = new_x;

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
            mvprintw(y/2-7, x/2-15, "Score: %d", score);
            mvprintw(y/2-7, x/2+8, "X drone pos: %d", drone_x);
            mvprintw(y/2-6, x/2+8, "Y drone pos: %d", drone_y);
            refresh();
        }
        if(input_key == 'w' || input_key == 'e' || input_key == 'r' || input_key == 's'|| input_key == 'd' || 
            input_key == 'f' || input_key == 'x' || input_key == 'c' || input_key == 'v' || input_key == 'q'){
                write(fd_key, &input_key,1);
                if(input_key == 'q')
                    exit(EXIT_SUCCESS);
        }
    }   

    close(fd_key);
    close(fd_bb);

    getch();
    endwin();
    return 0;
}