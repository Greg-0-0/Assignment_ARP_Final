// gcc -Wall -Wextra blackboard.c -lncurses -o blackboard
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#define _XOPEN_SOURCE_EXTENDED
#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<ncurses.h>
#include <locale.h>

typedef enum{
    MSG_QUIT = 0,
    MSG_POS = 1,
    MSG_NPOS = 2,
    MSG_STOP = 3,
    MSG_NAN = 4
} MsgType;

typedef struct{
    MsgType type;
    int new_drone_y;
    int new_drone_x;
} DroneMsg;

typedef struct{
    MsgType type;
    int drone_y;
    int drone_x;
    int border_y;
    int border_x;
} BlackboardMsg;

void draw_rect(WINDOW *win, int y, int x, int h, int w, int color_pair)
{
    wattron(win, COLOR_PAIR(color_pair));

    // top and bottom
    for (int col = x; col < w; col++) {
        mvwaddch(win, y, col, ACS_HLINE);
        mvwaddch(win, h, col, ACS_HLINE);
    }

    // left and right
    for (int row = y; row < h; row++) {
        mvwaddch(win, row, x, ACS_VLINE);
        mvwaddch(win, row, w, ACS_VLINE);
    }

    // corners
    mvwaddch(win, y, x, ACS_ULCORNER);
    mvwaddch(win, y, w, ACS_URCORNER);
    mvwaddch(win, h, x, ACS_LLCORNER);
    mvwaddch(win, h, w, ACS_LRCORNER);

    wattroff(win, COLOR_PAIR(color_pair));

    wrefresh(win);
}


static void layout_and_draw(WINDOW *win) {
    int H, W;
    getmaxyx(stdscr, H, W);
    start_color();

    init_pair(1, COLOR_RED, COLOR_BLACK);

    // Windows with fixed margins
    int wh = (H > 6) ? H - 6 : H;
    int ww = (W > 10) ? W - 10 : W;
    if (wh < 3) wh = 3;
    if (ww < 3) ww = 3;

    // Resize and recenter the window
    wresize(win, wh, ww);
    mvwin(win, (H - wh) / 2, (W - ww) / 2);

    // Klean up and redraw
    werase(stdscr);
    werase(win);
    box(win, 0, 0);
    getmaxyx(win, H, W);
    // Drawable region goes from y:1 to H-2 and x:1 to W-2

    // Spawn drone
    mvwprintw(win,H/2,W/2,"+");

    refresh();
    wrefresh(win);

    draw_rect(win,6,6,H-7,W-7,1);
}

int main(int argc, char* argv[]) {
    /*
    // Receives request of new position from drone, non blocking otherwise it would block resize of window
    char* myfifo1 = "/tmp/myfifoFromDToBRequest";
    char* myfifo2 = "/tmp/myfifoFromBToD"; // Used to send current drone position, obstacle positions and borders position to drone
    // Receives new position of drone from drone (movement), can be blocking, since we are waiting for a new position, 
    // after pressing a key, and realistically we wouldn't choose to resize window at that moment
    char* myfifo3 = "/tmp/myfifoFromDToBNewPos";
    mkfifo(myfifo1, 0666);
    mkfifo(myfifo2, 0666);
    mkfifo(myfifo3, 0666);
    int fd1, fd2, fd3;
    */

    if(argc < 4){
        fprintf(stderr,"No arguments passed to blackboard\n");
        exit(EXIT_FAILURE);
    }

    int fd_req = atoi(argv[1]);
    int fd_npos = atoi(argv[2]);
    int fd_pos = atoi(argv[3]);

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

    int H, W;

    /*
    char input_string[80];
    char output_string[80];
    char positions_format[80] = "pos: %d,%d, borders: %d,%d";
    int drone_position[2]; // (y,x)
    int borders[2];
    */

    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0;
           positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_POS;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    setlocale(LC_ALL, "");

    initscr();
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
    
    /*
    fd1 = open(myfifo1, O_RDONLY | O_NONBLOCK);
    fd3 = open(myfifo3, O_RDONLY | O_NONBLOCK);
    fd2 = open(myfifo2, O_WRONLY);
    if(fd2 < 0){
        perror("open");
        return 0;
    }
    */

    refresh();

    // Saving borders value
    positions.border_y = H-7;
    positions.border_x = W-7;
    
    
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
        }
        //read(fd_req, input_string, sizeof(input_string)); // Checks for position request from drone (non blocking)
        read(fd_req, &drone_msg, sizeof(drone_msg)); // Checks for position request from drone (non blocking)
        if(drone_msg.type == MSG_POS/*input_string[0] == 'p'*/){
            // Reset string to avoid rexecuting if statement
            //strcpy(input_string,"");
            // Drone is asking for position
            //sprintf(output_string,positions_format, drone_position[0], drone_position[1], borders[0], borders[1]);
            //write(fd_pos,output_string,strlen(output_string)+1); // Sends current position
            write(fd_pos,&positions,sizeof(positions)); // Sends current position
            // If drone is asking for position it wants to move, thus reading on fd3
            while(1){
                //printf("try1\n");
                //read(fd_npos,output_string,sizeof(output_string)); // Receiving new position
                read(fd_npos,&drone_msg,sizeof(drone_msg)); // Receiving new position
                //printf("try2\n");
                if(drone_msg.type == MSG_STOP/*output_string[0] == 's'*/)
                    break;
                if(drone_msg.type == MSG_QUIT/*output_string[0] == 'q'*/)
                    exit(EXIT_SUCCESS);
                //mvwaddch(win,drone_position[0],drone_position[1],' ');
                mvwaddch(win,positions.drone_y,positions.drone_x,' ');
                //sscanf(output_string,positions_format,&drone_position[0],&drone_position[1],&borders[0],&borders[1]);
                positions.drone_y = drone_msg.new_drone_y;
                positions.drone_x = drone_msg.new_drone_x;
                //mvwprintw(win,drone_position[0],drone_position[1],"+");
                mvwprintw(win,positions.drone_y,positions.drone_x,"+");
                wrefresh(win);
            }
        }
        else if(drone_msg.type == MSG_QUIT/*input_string[0] == 'q'*/){
            exit(EXIT_SUCCESS);
        }
        wrefresh(win);
    }

    close(fd_req);
    close(fd_npos);
    close(fd_pos);

    delwin(win);
    endwin();
    return 0;
}