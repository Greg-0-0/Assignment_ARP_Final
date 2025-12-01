#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<math.h>
#include<time.h>
#include<sys/select.h>

#define N_OBS 10

typedef enum{
    MSG_QUIT = 0,
    MSG_POS = 1,
    MSG_NPOS = 2,
    MSG_STOP = 3,
    MSG_NAN = 4
} MsgType;

typedef struct{
    MsgType type;
    int drone_y;
    int drone_x;
    int border_y;
    int border_x;
    int obstacles[N_OBS][2];
} BlackboardMsg;

int main(int argc, char * argv[]){

    if(argc < 3){
        fprintf(stderr,"No arguments passed to obstacles\n");
        exit(EXIT_FAILURE);
    }

    int fd_new_pos = atoi(argv[1]); // Receives positions of drone and blackboard
    int fd_new_obs = atoi(argv[2]); // Sends position of obstacles
    BlackboardMsg positions; positions.type = MSG_NAN;
    
    srand(time(NULL));

    while(1){
        ssize_t n = read(fd_new_pos,&positions,sizeof(positions));
        if(n > 0){
            // Creating obstacles
            for(int i = 0;i<N_OBS;i++){
                int pos_y = 7 + rand() % (positions.border_y - 7); // Random generator from 7 to H - 8
                int pos_x = 7 + rand() % (positions.border_x - 7); // Random generator from 7 to W - 8
                positions.obstacles[i][0] = pos_y;
                positions.obstacles[i][1] = pos_x;
            }

            write(fd_new_obs,&positions,sizeof(positions));
        }
        else{
            printf("error\n");
            sleep(3);
            perror("read obstacles");
            exit(EXIT_FAILURE);
        }


    }
    return 0;

}