// gcc drone.c -o drone -lm
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

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

void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;  
    nanosleep(&ts, NULL);
}

//void move_drone(int fd_key, int fd_npos, int current_drone_pos[2], int previous_drone_pos[2], int next_drone_pos[2],double force_x,
//       double force_y, double max_force, double oblique_force_comp, double M, double K, double T, char pos_format[], int borders[]){
void move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y,
       double max_force, double oblique_force_comp, double M, double K, double T, int borders[]){

    fd_set rfds;
    struct timeval tv;
    int retval;
    int nfds = fd_key + 1;
    //char new_drone_pos[80];
    char received_key;
    char artifcial_key;
    //double loop_prev_drone_pos[2] = {previous_drone_pos[0], previous_drone_pos[1]};
    //double loop_curr_drone_pos[2] = {current_drone_pos[0], current_drone_pos[1]};
    // Initially current position and previous position are the same
    int previous_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_prev_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_curr_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_next_drone_pos[2] = {0.0, 0.0};
    double epsilon = 1e-3;
    double temp_x = 0.0;
    double temp_y = 0.0;
    int delay = 400, control = 0, is_on_border = 0;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // Select checks every 100 ms
    sleep_ms(250); // Delay to ensure multiple consecutive clicks are taken into account correctly (the forces sum up)

    do{
        FD_ZERO(&rfds);
        FD_SET(fd_key, &rfds);
        retval = select(nfds, &rfds, NULL, NULL, &tv); // Returns only if a key has been pressed
        if(retval == -1){
            perror("select");
            exit(EXIT_FAILURE);
        }
        else if((retval > 0 && FD_ISSET(fd_key, &rfds)) || is_on_border == 1){
            // Readable input
            ssize_t n = 0;
            // Checking if the drone is near the border, 
            if(is_on_border == 0)
                // No inputs user allowed, must exit the zone using artificial_key
                n = read(fd_key,&received_key,1);
            if(n > 0 || is_on_border == 1){
                //printf("try2\n");
                if(received_key == 'q'){
                    drone_msg->type = MSG_QUIT;
                    //write(fd_npos,&received_key,1);
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    exit(EXIT_SUCCESS);
                }
                // In case consecutive clicks are detected this delay allows for a smoother motion
                if(is_on_border == 1){
                    printf("detected %c\n", artifcial_key);
                    received_key = artifcial_key;
                }
                delay = 400;
                control = 0;
                switch (received_key)
                {
                case 'f':force_x = force_x + max_force; break;
                case 's':force_x = force_x - max_force; break;
                case 'e':force_y = force_y - max_force; break;
                case 'c':force_y = force_y + max_force; break;
                case 'w':
                    force_x = force_x - oblique_force_comp;
                    force_y = force_y - oblique_force_comp;
                    break;
                case 'r':
                    force_x = force_x + oblique_force_comp;
                    force_y = force_y - oblique_force_comp;
                    break;
                case 'x':
                    force_x = force_x - oblique_force_comp;
                    force_y = force_y + oblique_force_comp;
                    break;
                case 'v':
                    force_x = force_x + oblique_force_comp;
                    force_y = force_y + oblique_force_comp;
                    break;
                case 'd':
                    drone_msg->type = MSG_STOP;
                    //strcpy(new_drone_pos,"stop");
                    //write(fd_npos,new_drone_pos,strlen(new_drone_pos)+1);
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    return;
                default:
                    break;
                }
            }
        }
        received_key = 'o';
        control++;

        printf("applied force: %f\n", force_x);

        // Along x axis
        temp_x = force_x - (M/(T*T)*(loop_prev_drone_pos[1]-2*loop_curr_drone_pos[1])) + 
                loop_curr_drone_pos[1]*(K/T);
        loop_next_drone_pos[1] = temp_x/((M/(T*T))+(K/T));

        loop_prev_drone_pos[1] = loop_curr_drone_pos[1];
        loop_curr_drone_pos[1] = loop_next_drone_pos[1];

        next_drone_pos[1] = (int)round(loop_next_drone_pos[1]);

        //previous_drone_pos[1] = current_drone_pos[1];
        //current_drone_pos[1] = next_drone_pos[1];
        previous_drone_pos[1] = drone_msg->new_drone_x;
        drone_msg->new_drone_x = next_drone_pos[1];

        // Along y axis
        temp_y = force_y - (M/(T*T)*(loop_prev_drone_pos[0]-2*loop_curr_drone_pos[0])) + 
                loop_curr_drone_pos[0]*(K/T);
        loop_next_drone_pos[0] = temp_y/((M/(T*T))+(K/T));

        loop_prev_drone_pos[0] = loop_curr_drone_pos[0];
        loop_curr_drone_pos[0] = loop_next_drone_pos[0];

        next_drone_pos[0] = (int)round(loop_next_drone_pos[0]);

        //previous_drone_pos[0] = current_drone_pos[0];
        //current_drone_pos[0] = next_drone_pos[0];
        previous_drone_pos[0] = drone_msg->new_drone_y;
        drone_msg->new_drone_y = next_drone_pos[0];

        // New position is sent to blackboard for graphical update
        //sprintf(new_drone_pos,pos_format,next_drone_pos[0],next_drone_pos[1],borders[0],borders[1]);
        //write(fd_npos,new_drone_pos,strlen(new_drone_pos)+1);
        write(fd_npos,drone_msg,sizeof(*drone_msg));

        // Checking if drone is within 5 pixels from the border
        // controllo posizione drone precedente per sapere da dove sta arrivando e considero la chiave di input per andare nella direzione opposta evitando di considerare l'input da pipe ma prenderndolo comunque
        // resetto il delay e control
        //if((current_drone_pos[0] < 6 || current_drone_pos[1] < 6 || current_drone_pos[0] > borders[0] || current_drone_pos[1] > borders[1]) & is_on_border == 0){
        if((drone_msg->new_drone_y < 6 || drone_msg->new_drone_x < 6 || drone_msg->new_drone_y > borders[0] || drone_msg->new_drone_x > borders[1]) && is_on_border == 0){
            printf("border\n");
            is_on_border = 1;
            control = 0;
            delay = 400;
            force_x = 0.0;
            force_y = 0.0;

            // For every possible direction, when reaching the border, providing an opposing repulsive force
            //if(current_drone_pos[0] == previous_drone_pos[0]){
            if(drone_msg->new_drone_y == previous_drone_pos[0]){
                //printf("straight f, curr pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1]);
                printf("straight f, curr pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x);
                // Drone is moving along x axis
                //if(current_drone_pos[1] < 6)
                if(drone_msg->new_drone_x < 6)
                    // Reaching left end
                    artifcial_key = 'f'; // Input of the border (not using 'received_key' so eventual 'q' input is still active)
                else
                    // Reaching right end
                    artifcial_key = 's';

            }
            //else if(current_drone_pos[1] == previous_drone_pos[1]){
            else if(drone_msg->new_drone_x == previous_drone_pos[1]){
                //printf("straight s, curr pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1]);
                printf("straight s, curr pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x);
                // Drone is moving along y axis
                 //if(current_drone_pos[0] < 6)
                 if(drone_msg->new_drone_y < 6)
                    // Reaching upper end
                    artifcial_key = 'c';
                else
                    // Reaching lower end
                    artifcial_key = 'e';
            }
            else{
                // Drone is moving diagonally
                //if(current_drone_pos[0] > previous_drone_pos[0] && current_drone_pos[1] > previous_drone_pos[1])
                if(drone_msg->new_drone_y > previous_drone_pos[0] && drone_msg->new_drone_x > previous_drone_pos[1])
                    //printf("straight v, curr pos: %d,%d, prev pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1], previous_drone_pos[0],previous_drone_pos[1]);
                    printf("straight v, curr pos: %d,%d, prev pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x, previous_drone_pos[0],previous_drone_pos[1]);
                    // Moving towards bottom-right corner
                    //if(current_drone_pos[0] > borders[0])
                    if(drone_msg->new_drone_y > borders[0])
                        // Reaching lower end
                        artifcial_key = 'r';
                    //else if(current_drone_pos[1] > borders[1])
                    else if(drone_msg->new_drone_x > borders[1])
                        // Reaching right end
                        artifcial_key = 'x';
                //if(current_drone_pos[0] > previous_drone_pos[0] && current_drone_pos[1] < previous_drone_pos[1]){
                if(drone_msg->new_drone_y > previous_drone_pos[0] && drone_msg->new_drone_x < previous_drone_pos[1]){
                    //printf("straight x, curr pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1]);
                    printf("straight x, curr pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x);
                    // Moving towards bottom-left corner
                    //if(current_drone_pos[0] > borders[0])
                    if(drone_msg->new_drone_y > borders[0])
                        // Reaching lower end
                        artifcial_key = 'w';
                    else
                        // Reaching left end
                        artifcial_key = 'v';
                }
                //if(current_drone_pos[0] < previous_drone_pos[0] && current_drone_pos[1] < previous_drone_pos[1]){
                if(drone_msg->new_drone_y < previous_drone_pos[0] && drone_msg->new_drone_x < previous_drone_pos[1]){
                    //printf("straight w, curr pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1]);
                    printf("straight w, curr pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x);
                    // Moving towards top-left corner
                    //if(current_drone_pos[0] < 6)
                    if(drone_msg->new_drone_y < 6)
                        // Reaching upper end
                        artifcial_key = 'x';
                    else
                        // Reaching left end
                        artifcial_key = 'r';
                }
                //if(current_drone_pos[0] < previous_drone_pos[0] && current_drone_pos[1] > previous_drone_pos[1]){
                if(drone_msg->new_drone_y < previous_drone_pos[0] && drone_msg->new_drone_x > previous_drone_pos[1]){
                    //printf("straight r, curr pos: %d,%d\n",current_drone_pos[0],current_drone_pos[1]);
                    printf("straight r, curr pos: %d,%d\n",drone_msg->new_drone_y,drone_msg->new_drone_x);
                    // Moving towards top-right corner
                    //if(current_drone_pos[0] < 6)
                    if(drone_msg->new_drone_y < 6)
                        // Reaching upper end
                        artifcial_key = 'v';
                    else
                        // Reaching right end
                        artifcial_key = 'w';
                }
            }
        }
        else
            is_on_border = 0;

        // Delay inserted to simulate a smoother slow down
        if(control == 4) delay = 50;
        sleep_ms(delay);

        // After first impulse forces are resetted
        force_x = 0.0;
        force_y = 0.0;

    }while(fabs(loop_next_drone_pos[1] - loop_prev_drone_pos[1]) > epsilon || 
            fabs(loop_next_drone_pos[0] - loop_prev_drone_pos[0]) > epsilon);
    
    // Signaling blackboard to interrupt waiting for new postions
    //strcpy(new_drone_pos,"stop");
    //write(fd_npos,new_drone_pos,strlen(new_drone_pos)+1);
    drone_msg->type = MSG_STOP;
    write(fd_npos,drone_msg,sizeof(*drone_msg));
    
    return;
}

int main(int argc, char* argv[]){

    /*
    // Receives pressed key from input_manager, blocking since drone doesn't have to carry out other actions 
    char* myfifo1 = "/tmp/myfifoFromIToD";
    // Requests current drone position and obstacles position from blackboard
    char* myfifo2 = "/tmp/myfifoFromDToBRequest";
    char* myfifo3 = "/tmp/myfifoFromBToD"; // Receives current drone position and obstacles position from blackboard
    char* myfifo4 = "/tmp/myfifoFromDToBNewPos"; // Sends new drone position to blackboard
    mkfifo(myfifo1, 0666);
    mkfifo(myfifo2, 0666);
    mkfifo(myfifo3, 0666);
    mkfifo(myfifo4, 0666);
    int fd1, fd2, fd3, fd4;
    */

    if(argc < 4){
        fprintf(stderr,"No arguments passed to drone\n");
        exit(EXIT_FAILURE);
    }

    int fd_req = atoi(argv[1]);
    int fd_npos = atoi(argv[2]);
    int fd_pos = atoi(argv[3]);
    int fd_key = atoi(argv[4]);

    char received_key;
    //char request_pos[80] = "pos";
    //char input_string[80];
    char stop_char = '-';
    //char new_drone_position[80];
    //char positions_format[80] = "pos: %d,%d, borders: %d, %d";
    int next_drone_position[2] = {0,0};
    //int current_drone_position[2] = {0,0};
    //int previous_drone_position[2] = {0,0};
    int borders[2] = {0,0};
    double max_applied_force = 2.55;
    double force_on_x = 0.0;
    double force_on_y = 0.0;
    double oblique_force_comp = (sqrt(2)/2)*max_applied_force;
    const double M = 1.0;
    const double K = 1.0;
    const double T = 1;
    int first = 1;

    BlackboardMsg blackboard_msg;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    /*
    fd4 = open(myfifo4,O_WRONLY); // Sends new drone position
    if(fd4 < 0){
        perror("open");
        return 0;
    }
    fd1 = open(myfifo1,O_RDONLY); // Input key received
    fd2 = open(myfifo2,O_WRONLY); // Requests drone/obstacles position
    if(fd2 < 0){
        perror("open");
        return 0;
    }
    fd3 = open(myfifo3,O_RDONLY); // Returns drone/obstacles position
    */

    while(1){
        int n = read(fd_key,&received_key,1);
        printf("Bytes read: %d\n", n);
        if(n > 0){
            printf("Bytes read: %d\n", n);
            if(received_key == 'q'){
                printf("Bytes read: %d\n", n);
                drone_msg.type = MSG_QUIT;
                //write(fd_req,&received_key,1);
                write(fd_req,&drone_msg,sizeof(drone_msg));
                printf("Bytes read: %d\n", n);
                exit(EXIT_SUCCESS);
            }
            drone_msg.type = MSG_POS;
            //write(fd_req, request_pos, strlen(request_pos)+1); // Requests drone, obstacles and borders positions
            write(fd_req, &drone_msg, sizeof(drone_msg)); // Requests drone, obstacles and borders positions
            //read(fd_pos, input_string, sizeof(input_string)); // Receives positions
            read(fd_pos, &blackboard_msg, sizeof(blackboard_msg)); // Receives positions
            //sscanf(input_string, positions_format, &current_drone_position[0], &current_drone_position[1], &borders[0], &borders[1]);
            drone_msg.new_drone_y = blackboard_msg.drone_y;
            drone_msg.new_drone_x = blackboard_msg.drone_x;
            borders[0] = blackboard_msg.border_y;
            borders[1] = blackboard_msg.border_x;
            // Since it's at rest
            //previous_drone_position[0] = current_drone_position[0];
            //previous_drone_position[1] = current_drone_position[1];
            switch (received_key)
            {
                // Updating drone position using Euler's method
            case 'f': force_on_x = max_applied_force; force_on_y = 0.0;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes to the left (x changes)
                break;
            case 's': force_on_x = -max_applied_force; force_on_y = 0.0;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes to the left (x changes)
                break;
            case 'e': force_on_x = 0.0; force_on_y = -max_applied_force;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes up (y changes)
                break;
            case 'c': force_on_x = 0.0; force_on_y = max_applied_force;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes down (y changes)
                break;
            case 'w': force_on_x = -oblique_force_comp; force_on_y = -oblique_force_comp;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes to left/up
                break;
            case 'r': force_on_x = oblique_force_comp; force_on_y = -oblique_force_comp;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes right/up
                break;
            case 'x': force_on_x = -oblique_force_comp; force_on_y = oblique_force_comp;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes left/down
                break;
            case 'v': force_on_x = oblique_force_comp; force_on_y = oblique_force_comp;
                    //move_drone(fd_key,fd_npos,current_drone_position,previous_drone_position,next_drone_position,
                    //force_on_x,force_on_y,max_applied_force,oblique_force_comp,M,K,T,positions_format,borders);
                    move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                    force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders);
                // Drone goes right/down
                break;
            case 'q':
                drone_msg.type = MSG_QUIT;
                //write(fd_req,&received_key,1);
                write(fd_req,&drone_msg,sizeof(drone_msg));
                exit(EXIT_SUCCESS);
            default:
                break;
            }
    
            drone_msg.new_drone_y = next_drone_position[0];
            drone_msg.new_drone_x = next_drone_position[1];
            //current_drone_position[0] = next_drone_position[0];
            //current_drone_position[1] = next_drone_position[1];
            drone_msg.type = MSG_NAN;
        }        
    }
    close(fd_req);
    close(fd_npos);
    close(fd_pos);
    close(fd_key);

    return 0;
}