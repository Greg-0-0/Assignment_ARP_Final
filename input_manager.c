// gcc input_manager.c -lncurses -o input_manager
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<ncurses.h>

int main(int argc, char* argv[]){

    if(argc < 2){
        fprintf(stderr,"No arguments passed to input manager\n");
        exit(EXIT_FAILURE);
    }

    int fd_key = atoi(argv[1]);

    int y, x;

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
    refresh();

    while(1){
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

    getch();
    endwin();
    return 0;
}