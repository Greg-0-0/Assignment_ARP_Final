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
    int new_drone_y;
    int new_drone_x;
} DroneMsg;

typedef struct{
    MsgType type;
    int drone_y;
    int drone_x;
    int border_y;
    int border_x;
    int obstacles[N_OBS][2];
} BlackboardMsg;

void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;  
    nanosleep(&ts, NULL);
}

void drain_pipe(int fd) {
    // Set pipe to non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    char buf[256];
    while (read(fd, buf, sizeof(buf)) > 0) {
        // Keep reading until pipe is empty
    }

    // Restore original flags
    fcntl(fd, F_SETFL, flags);
}

void move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y,
       double max_force, double oblique_force_comp, double M, double K, double T, int borders[], char last_valid_key){

    fd_set rfds;
    struct timeval tv;
    int retval;
    int nfds = fd_key + 1;
    char received_key, artifcial_key;
    int previous_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_prev_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_curr_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_next_drone_pos[2] = {0.0, 0.0};
    double epsilon = 1e-3;
    double temp_x = 0.0;
    double temp_y = 0.0;
    int delay = 40, control = 0, is_on_border = 0, first = 0;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // Select checks every 100 ms
    sleep_ms(250); // Delay to ensure multiple consecutive clicks are taken into account correctly (the forces sum up)

    do{
        printf("las %c\n",last_valid_key);
        FD_ZERO(&rfds);
        FD_SET(fd_key, &rfds);
        retval = select(nfds, &rfds, NULL, NULL, &tv); // Returns only if a key has been pressed
        if(retval == -1){
            perror("select");
            exit(EXIT_FAILURE);
        }
        else if(retval > 0 && FD_ISSET(fd_key, &rfds)){
            // Readable input
            ssize_t n = 0;
            n = read(fd_key,&received_key,1);
            if(n > 0){
                if(received_key == 'q'){
                    drone_msg->type = MSG_QUIT;
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    exit(EXIT_SUCCESS);
                }
                delay = 40;
                control = 0;
                switch (received_key)
                {
                case 'f':force_x = force_x + max_force; break;
                case 's':force_x = force_x - max_force; 
                    printf("force: %f\n", force_x);
                    break;
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
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    return;
                default:
                    break;
                }
            }
        }
        control++;

        // Along x axis
        temp_x = force_x - (M/(T*T)*(loop_prev_drone_pos[1]-2*loop_curr_drone_pos[1])) + 
                loop_curr_drone_pos[1]*(K/T);
        loop_next_drone_pos[1] = temp_x/((M/(T*T))+(K/T));

        loop_prev_drone_pos[1] = loop_curr_drone_pos[1];
        loop_curr_drone_pos[1] = loop_next_drone_pos[1];

        next_drone_pos[1] = (int)round(loop_next_drone_pos[1]);

        previous_drone_pos[1] = drone_msg->new_drone_x;
        drone_msg->new_drone_x = next_drone_pos[1];

        // Along y axis
        temp_y = force_y - (M/(T*T)*(loop_prev_drone_pos[0]-2*loop_curr_drone_pos[0])) + 
                loop_curr_drone_pos[0]*(K/T);
        loop_next_drone_pos[0] = temp_y/((M/(T*T))+(K/T));

        loop_prev_drone_pos[0] = loop_curr_drone_pos[0];
        loop_curr_drone_pos[0] = loop_next_drone_pos[0];

        next_drone_pos[0] = (int)round(loop_next_drone_pos[0]);

        previous_drone_pos[0] = drone_msg->new_drone_y;
        drone_msg->new_drone_y = next_drone_pos[0];

        // New position is sent to blackboard for graphical update
        write(fd_npos,drone_msg,sizeof(*drone_msg));

        // Checking if drone is within 5 pixels from the border
        while(drone_msg->new_drone_y <= 6 || drone_msg->new_drone_x <= 6 || 
            drone_msg->new_drone_y >= borders[0] || drone_msg->new_drone_x >= borders[1]){
            printf("border\n");
            is_on_border = 1;
            control = 0;
            delay = 150;

            printf("assinging last %c\n",last_valid_key);
            //received_key = last_valid_key;

            switch (received_key)
            {
            case 'f': 
                printf("applying new key s\n");
                printf("x pos %d\n",drone_msg->new_drone_x);
                printf("y pos %d\n",drone_msg->new_drone_y);
                artifcial_key = 's'; 
                if((drone_msg->new_drone_x >= borders[1]) && (drone_msg->new_drone_x < (borders[1]+2)))
                    force_x = force_x - (max_force/2); 
                else if((drone_msg->new_drone_x >= (borders[1]+2)) && (drone_msg->new_drone_x < (borders[1]+4)))
                    force_x = force_x - max_force - 1;
                else
                    force_x = force_x - (2*max_force) - 1;
                break;
            case 's': 
                artifcial_key = 'f';
                if((drone_msg->new_drone_x <= 6) && (drone_msg->new_drone_x > 4))
                    force_x = force_x + (max_force/2);
                else if((drone_msg->new_drone_x <= 4) && (drone_msg->new_drone_x > 2))
                    force_x = force_x + max_force + 1;
                else
                    force_x = force_x + (2*max_force) + 1;
                break;
            case 'e': artifcial_key = 'c'; 
                if((drone_msg->new_drone_y <= 6) && (drone_msg->new_drone_y > 4))
                    force_y = force_y + (max_force/2);
                else if((drone_msg->new_drone_y <= 4)  && (drone_msg->new_drone_y > 2))
                    force_y = force_y + max_force;
                else
                    force_y = force_y + (2*max_force);
                break;
            case 'c': artifcial_key = 'e'; 
                if((drone_msg->new_drone_y >= borders[0]) && (drone_msg->new_drone_y < (borders[0]+2)))
                    force_y = force_y - (max_force/2);
                else if((drone_msg->new_drone_y >= (borders[0]+2)) && (drone_msg->new_drone_y < borders[0]+4))
                    force_y = force_y - max_force;
                else
                    force_y = force_y - (2*max_force);
                break;
            case 'r': //------------------------------- need a check
                printf("r detected\n");
                printf("x pos %d\n",drone_msg->new_drone_x);
                printf("y pos %d\n",drone_msg->new_drone_y);
                if(drone_msg->new_drone_y <= 6){
                    printf("executing x or v\n");
                    {
                        printf("upper border taken\n");
                        artifcial_key = 'v';
                        if((drone_msg->new_drone_y <= 6) && (drone_msg->new_drone_y > 4)){
                            force_x = force_x + (oblique_force_comp/2);
                            force_y = force_y + (oblique_force_comp/2);
                        }
                        else if((drone_msg->new_drone_y <= 4)  && (drone_msg->new_drone_y > 2)){
                            force_x = force_x + oblique_force_comp;
                            force_y = force_y + oblique_force_comp;
                        }
                        else{
                            force_x = force_x + (2*oblique_force_comp);
                            force_y = force_y + (2*oblique_force_comp);
                        }
                    }
                }
                else if(drone_msg->new_drone_x >= borders[1]){
                    printf("right border taken\n");
                    artifcial_key = 'w';
                    if((drone_msg->new_drone_x >= borders[1]) && (drone_msg->new_drone_x < (borders[1]+2))){
                        printf("forces applied\n");
                        force_x = force_x - (oblique_force_comp/2) - 1;
                        force_y = force_y - (oblique_force_comp/2);
                    } 
                    else if((drone_msg->new_drone_x >= (borders[1]+2)) && (drone_msg->new_drone_x < (borders[1]+4))){
                        force_x = force_x - oblique_force_comp - 1;
                        force_y = force_y - oblique_force_comp;
                    }
                    else{
                        force_x = force_x - (2*oblique_force_comp);
                        force_y = force_y - (2*oblique_force_comp);
                    }
                }
                printf("after else if\n");
                break;
            case 'v': //-------------------------------
                if(drone_msg->new_drone_y >= borders[0]){
                    artifcial_key = 'r';
                    if((drone_msg->new_drone_y >= borders[0]) && (drone_msg->new_drone_y < (borders[0]+2))){
                        force_x = force_x + (oblique_force_comp/2);
                        force_y = force_y - (oblique_force_comp/2);
                    }
                    else if((drone_msg->new_drone_y >= (borders[0]+2)) && (drone_msg->new_drone_y < borders[0]+4)){
                        force_x = force_x + oblique_force_comp;
                        force_y = force_y - oblique_force_comp;
                    }
                    else{
                        force_x = force_x + (2*oblique_force_comp);
                        force_y = force_y - (2*oblique_force_comp);
                    }
                }
                else if(drone_msg->new_drone_x >= borders[1]){
                    artifcial_key = 'x';
                    if((drone_msg->new_drone_x >= borders[1]) && (drone_msg->new_drone_x < (borders[1]+2))){
                        force_x = force_x - (oblique_force_comp/2) - 1;
                        force_y = force_y + (oblique_force_comp/2);
                    } 
                    else if((drone_msg->new_drone_x >= (borders[1]+2)) && (drone_msg->new_drone_x < (borders[1]+4))){
                        force_x = force_x - oblique_force_comp - 1;
                        force_y = force_y + oblique_force_comp;
                    }
                    else{
                        force_x = force_x - (2*oblique_force_comp);
                        force_y = force_y + (2*oblique_force_comp);
                    }
                }
                break;
            case 'x': 
                if(drone_msg->new_drone_y >= borders[0]){
                    artifcial_key = 'w';
                    if((drone_msg->new_drone_y >= borders[0]) && (drone_msg->new_drone_y < (borders[0]+2))){
                        force_x = force_x - (oblique_force_comp/2);
                        force_y = force_y - (oblique_force_comp/2);
                    }
                    else if((drone_msg->new_drone_y >= (borders[0]+2)) && (drone_msg->new_drone_y < borders[0]+4)){
                        force_x = force_x - oblique_force_comp;
                        force_y = force_y - oblique_force_comp;
                    }
                    else{
                        force_x = force_x - (2*oblique_force_comp);
                        force_y = force_y - (2*oblique_force_comp);
                    }
                }
                else if(drone_msg->new_drone_x <= 6){
                    artifcial_key = 'v';
                    if((drone_msg->new_drone_x <= 6) && (drone_msg->new_drone_x > 4)){
                        force_x = force_x + (oblique_force_comp/2);
                        force_y = force_y + (oblique_force_comp/2);
                    }
                    else if((drone_msg->new_drone_x <= 4) && (drone_msg->new_drone_x > 2)){
                        force_x = force_x + oblique_force_comp;
                        force_y = force_y + oblique_force_comp;
                    }
                    else{
                        force_x = force_x + (2*oblique_force_comp);
                        force_y = force_y + (2*oblique_force_comp);
                    }
                }
                break;
            case 'w':
                if(drone_msg->new_drone_y <= 6){
                    {
                        artifcial_key = 'x';
                        if((drone_msg->new_drone_y <= 6) && (drone_msg->new_drone_y > 4)){
                            force_x = force_x - (oblique_force_comp/2);
                            force_y = force_y + (oblique_force_comp/2);
                        }
                        else if((drone_msg->new_drone_y <= 4)  && (drone_msg->new_drone_y > 2)){
                            force_x = force_x - oblique_force_comp;
                            force_y = force_y + oblique_force_comp;
                        }
                        else{
                            force_x = force_x - (2*oblique_force_comp);
                            force_y = force_y + (2*oblique_force_comp);
                        }
                    }
                }
                else if(drone_msg->new_drone_x <= 6){
                    artifcial_key = 'r';
                    if((drone_msg->new_drone_x <= 6) && (drone_msg->new_drone_x > 4)){
                        force_x = force_x + (oblique_force_comp/2);
                        force_y = force_y - (oblique_force_comp/2);
                    }
                    else if((drone_msg->new_drone_x <= 4) && (drone_msg->new_drone_x > 2)){
                        force_x = force_x + oblique_force_comp;
                        force_y = force_y - oblique_force_comp;
                    }
                    else{
                        force_x = force_x + (2*oblique_force_comp);
                        force_y = force_y - (2*oblique_force_comp);
                    }
                }
                break;
            default:
                break;
            }

            printf("artificial key %c\n", artifcial_key);
            printf("applying force %f\n", force_x);
            // Along x axis
            temp_x = force_x - (M/(T*T)*(loop_prev_drone_pos[1]-2*loop_curr_drone_pos[1])) + 
                    loop_curr_drone_pos[1]*(K/T);
            loop_next_drone_pos[1] = temp_x/((M/(T*T))+(K/T));

            loop_prev_drone_pos[1] = loop_curr_drone_pos[1];
            loop_curr_drone_pos[1] = loop_next_drone_pos[1];

            next_drone_pos[1] = (int)round(loop_next_drone_pos[1]);

            previous_drone_pos[1] = drone_msg->new_drone_x;
            drone_msg->new_drone_x = next_drone_pos[1];

            // Along y axis
            temp_y = force_y - (M/(T*T)*(loop_prev_drone_pos[0]-2*loop_curr_drone_pos[0])) + 
                    loop_curr_drone_pos[0]*(K/T);
            loop_next_drone_pos[0] = temp_y/((M/(T*T))+(K/T));

            loop_prev_drone_pos[0] = loop_curr_drone_pos[0];
            loop_curr_drone_pos[0] = loop_next_drone_pos[0];

            next_drone_pos[0] = (int)round(loop_next_drone_pos[0]);

            previous_drone_pos[0] = drone_msg->new_drone_y;
            drone_msg->new_drone_y = next_drone_pos[0];

            // New position is sent to blackboard for graphical update
            write(fd_npos,drone_msg,sizeof(*drone_msg));

            sleep_ms(delay);

            control++;

            if(force_x > 0)
                force_x -= 1;
            if(force_x < 0)
                force_x += 1;
            if(force_y > 0)
                force_y -= 1;
            if(force_y < 0)
                force_y += 1;
        }
        if(fabs(force_x) > epsilon || fabs(force_y) > epsilon){
            // Along x axis
            temp_x = force_x - (M/(T*T)*(loop_prev_drone_pos[1]-2*loop_curr_drone_pos[1])) + 
                    loop_curr_drone_pos[1]*(K/T);
            loop_next_drone_pos[1] = temp_x/((M/(T*T))+(K/T));

            loop_prev_drone_pos[1] = loop_curr_drone_pos[1];
            loop_curr_drone_pos[1] = loop_next_drone_pos[1];

            next_drone_pos[1] = (int)round(loop_next_drone_pos[1]);

            previous_drone_pos[1] = drone_msg->new_drone_x;
            drone_msg->new_drone_x = next_drone_pos[1];

            // Along y axis
            temp_y = force_y - (M/(T*T)*(loop_prev_drone_pos[0]-2*loop_curr_drone_pos[0])) + 
                    loop_curr_drone_pos[0]*(K/T);
            loop_next_drone_pos[0] = temp_y/((M/(T*T))+(K/T));

            loop_prev_drone_pos[0] = loop_curr_drone_pos[0];
            loop_curr_drone_pos[0] = loop_next_drone_pos[0];

            next_drone_pos[0] = (int)round(loop_next_drone_pos[0]);

            previous_drone_pos[0] = drone_msg->new_drone_y;
            drone_msg->new_drone_y = next_drone_pos[0];

            // New position is sent to blackboard for graphical update
            write(fd_npos,drone_msg,sizeof(*drone_msg));

            sleep_ms(delay);

            control++;

            if(force_x > 0)
                force_x -= 1;
            if(force_x < 0)
                force_x += 1;
            if(force_y > 0)
                force_y -= 1;
            if(force_y < 0)
                force_y += 1;
        }
        
        if(is_on_border == 1){
            drone_msg->type = MSG_STOP;
            write(fd_npos,drone_msg,sizeof(*drone_msg));
            drain_pipe(fd_key);
            return;
        }
        // Delay inserted to simulate a smoother slow down
        if(control == 4) delay = 70;
        sleep_ms(delay);

        // After first impulse forces are resetted
        force_x = 0.0;
        force_y = 0.0;

        first = 1;

    }while(fabs(loop_next_drone_pos[1] - loop_prev_drone_pos[1]) > epsilon || 
            fabs(loop_next_drone_pos[0] - loop_prev_drone_pos[0]) > epsilon);
    
    // Signaling blackboard to interrupt waiting for new postions
    drone_msg->type = MSG_STOP;
    write(fd_npos,drone_msg,sizeof(*drone_msg));
    
    return;
}

int main(int argc, char* argv[]){

    if(argc < 4){
        fprintf(stderr,"No arguments passed to drone\n");
        exit(EXIT_FAILURE);
    }

    int fd_req = atoi(argv[1]);
    int fd_npos = atoi(argv[2]);
    int fd_pos = atoi(argv[3]);
    int fd_key = atoi(argv[4]);

    char received_key;
    char stop_char = '-';
    int next_drone_position[2] = {0,0};
    int borders[2] = {0,0};
    double max_applied_force = 2.55;
    double force_on_x = 0.0;
    double force_on_y = 0.0;
    double oblique_force_comp = (sqrt(2)/2)*max_applied_force;
    const double M = 1.0;
    const double K = 1.0;
    const double T = 1;

    BlackboardMsg blackboard_msg;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    while(1){
        int n = read(fd_key,&received_key,1);
        if(n > 0){
            if(received_key == 'q'){
                drone_msg.type = MSG_QUIT;
                write(fd_req,&drone_msg,sizeof(drone_msg));
                exit(EXIT_SUCCESS);
            }
            if(received_key == 'f' || received_key == 's' || received_key == 'e' || received_key == 'c' || 
                received_key == 'w' || received_key == 'r' || received_key == 'x' || received_key == 'v'){
                drone_msg.type = MSG_POS;
                write(fd_req, &drone_msg, sizeof(drone_msg)); // Requests drone, obstacles and borders positions
                read(fd_pos, &blackboard_msg, sizeof(blackboard_msg)); // Receives positions
                drone_msg.new_drone_y = blackboard_msg.drone_y;
                drone_msg.new_drone_x = blackboard_msg.drone_x;
                borders[0] = blackboard_msg.border_y;
                borders[1] = blackboard_msg.border_x;
                // Since it's at rest
                switch (received_key)
                {
                    // Updating drone position using Euler's method
                case 'f': force_on_x = max_applied_force; force_on_y = 0.0;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'f');
                    // Drone goes to the left (x changes)
                    break;
                case 's': force_on_x = -max_applied_force; force_on_y = 0.0;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'s');
                    // Drone goes to the left (x changes)
                    break;
                case 'e': force_on_x = 0.0; force_on_y = -max_applied_force;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'e');
                    // Drone goes up (y changes)
                    break;
                case 'c': force_on_x = 0.0; force_on_y = max_applied_force;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'c');
                    // Drone goes down (y changes)
                    break;
                case 'w': force_on_x = -oblique_force_comp; force_on_y = -oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'w');
                    // Drone goes to left/up
                    break;
                case 'r': force_on_x = oblique_force_comp; force_on_y = -oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'r');
                    // Drone goes right/up
                    break;
                case 'x': force_on_x = -oblique_force_comp; force_on_y = oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'x');
                    // Drone goes left/down
                    break;
                case 'v': force_on_x = oblique_force_comp; force_on_y = oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,
                        force_on_y,max_applied_force,oblique_force_comp,M,K,T,borders,'v');
                    // Drone goes right/down
                    break;
                case 'q':
                    drone_msg.type = MSG_QUIT;
                    write(fd_req,&drone_msg,sizeof(drone_msg));
                    exit(EXIT_SUCCESS);
                default:
                    drone_msg.type = MSG_NAN;
                    write(fd_npos,&drone_msg,sizeof(drone_msg));
                    break;
                }
                drone_msg.new_drone_y = next_drone_position[0];
                drone_msg.new_drone_x = next_drone_position[1];
                drone_msg.type = MSG_NAN;
            }
        }
    }
    close(fd_req);
    close(fd_npos);
    close(fd_pos);
    close(fd_key);

    return 0;
}