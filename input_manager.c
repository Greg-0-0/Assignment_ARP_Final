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
    /*
    char* myfifo = "/tmp/myfifoFromIToD";
    mkfifo(myfifo, 0666);
    int fd;
    */

    if(argc < 2){
        fprintf(stderr,"No arguments passed to input manager\n");
        exit(EXIT_FAILURE);
    }

    int fd_key = atoi(argv[1]);

    int x, y;

    char input_key = 'o';
    initscr();
    cbreak();
    noecho();
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
    refresh();

    /*
    fd = open(myfifo, O_WRONLY);
    if(fd < 0){
        // Careful executing this process before drone, beacuse it will return instantly,
        // since drone hasn't started yet, thus the open with write flag on this fd will return < 0
        // (the open with read flag on the same fd, that should be executed by drone, hasn't started yet)

        // Master process must execute drone before input_manager
        perror("open");
        return 0;
    }
        */

    while(1){
        input_key = getch();
        refresh();
        if(input_key == 'w' || input_key == 'e' || input_key == 'r' || input_key == 's'|| input_key == 'd' || 
            input_key == 'f' || input_key == 'x' || input_key == 'c' || input_key == 'v' || input_key == 'q'){
                write(fd_key, &input_key,1);
                if(input_key == 'q')
                    exit(EXIT_SUCCESS);
            }
        refresh();
    }   

    close(fd_key);

    getch();
    endwin();
    return 0;
}