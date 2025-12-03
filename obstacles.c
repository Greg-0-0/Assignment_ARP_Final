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

// To ensure all the bytes inside the received struct are read
ssize_t read_full(int fd, void* buf, size_t size) {
    size_t total = 0;
    while (total < size) {
        ssize_t n = read(fd, (char*)buf + total, size - total);
        if (n <= 0) return n; // error or pipe closed
        total += n;
    }
    return total;
}

int main(int argc, char * argv[]){

    if(argc < 3){
        fprintf(stderr,"No arguments passed to obstacles\n");
        exit(EXIT_FAILURE);
    }

    int fd_new_pos = atoi(argv[1]); // Receives positions of drone and blackboard
    int fd_new_obs = atoi(argv[2]); // Sends position of obstacles
    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0; 
    positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_NAN;
    
    srand(time(NULL));

    while(1){
        ssize_t n = read_full(fd_new_pos,&positions,sizeof(positions));
        // The call may read bytes not belonging to the same struct (BlackboardMsg), due to race conditions on pipe creating junk inside the pipe,
        // or sending different struct on the same pipe.
        // Just to be clear my code doesn't seem to cause these race conditions, neither it uses the same file descriptors to send different type of structures,
        // however, this did happen, and, as it took place, it vanished once I manually checked the number of bytes read and expected:
        // printf("Read bytes: %ld, expected: %ld", n, sizeof(positions));
        // which of course is quite odd, other than frustrating, since it shouldn't have fixed the issue.
        // This happens because the error itself is an event, I would say, completely unpredictable, that depends on the integrity of pipes themself,
        // which, unfortunatley, I am forced to use.
        
        if(n > 0){
            if(positions.type == MSG_QUIT)
                exit(EXIT_SUCCESS);
            printf("pos border: %d, %d\n", positions.border_x,positions.border_y);
            // Creating obstacles
            for(int i = 0;i<N_OBS;i++){
                printf("try3\n");
                int pos_y = 7 + rand() % (positions.border_y - 7); // Random generator from 7 to H - 8
                printf("try4\n");
                int pos_x = 7 + rand() % (positions.border_x - 7); // Random generator from 7 to W - 8
                printf("try5\n");
                positions.obstacles[i][0] = pos_y;
                positions.obstacles[i][1] = pos_x;
            }

            write(fd_new_obs,&positions,sizeof(positions));
        }
        else{
            perror("read obstacles");
            exit(EXIT_FAILURE);
        }


    }
    return 0;

}