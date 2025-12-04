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
#include <ctype.h>

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

// Scanning a string to remove its white spaces
static char* remove_white_space(char* string){
    while (isspace((unsigned char)*string))
        string++;
    if(*string == 0)
        return string;
    
    char* end = string + strlen(string) - 1;
    while(end > string && isspace((unsigned char)* end))
        end--;
    end[1] = '\0';
    return string;
}

// Copying parameter values on local variables
int load_parameters(const char* filename, double* drone_mass, double* air_resistance, double* integr_inter, int* rep_radius, double* max_force){
    FILE* file = fopen(filename, "r");
    if(!file){
        perror("file open");
        exit(EXIT_FAILURE);
    }

    char line[250];

    // Reading file line by line
    while(fgets(line,sizeof(line), file)){
        
        // Removing white spaces
        char* trimmed_line = remove_white_space(line);

        // Skipping comments and empty lines
        if(*trimmed_line == '#' || *trimmed_line == '\0')
            continue;
        
        char key[128];
        char value[128];

        // Parsing "KEY = VALUE" line format using key and values with a length up to 127 characters
        if(sscanf(trimmed_line, "%127[^=]=%127s", key, value) == 2){
            char* k = remove_white_space(key);
            char* v = remove_white_space(value);

            // Matching keys
            if(strcmp(k,"DRONE_MASS") == 0){
                *drone_mass = atoi(v);
                if(v < 0){
                    printf("Invalid mass value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"AIR_RESISTANCE") == 0){
                *air_resistance = atoi(v);
                if(v < 0){
                    printf("Invalid air resistance value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"INTEGRATION_INTERVAL") == 0){
                *integr_inter = atoi(v);
                if(v < 0){
                    printf("Invalid integration interval\n");
                    return -1;
                }
            }
            else if(strcmp(k,"REPULSIVE_RADIUS") == 0){
                *rep_radius = atoi(v);
                if(v < 0){
                    printf("Invalid repulsive radius\n");
                    return -1;
                }
            }
            else if(strcmp(k,"MAX_APPLIED_FORCE") == 0){
                *max_force = atoi(v);
                if(v < 0){
                    printf("Invalid maximum force value\n");
                    return -1;
                }
            }
            else{
                printf("Error: unknown parameter -> %s\n", k);
                return -1;
            }
        }
    }
    fclose(file);
    return 0;
}

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

void compute_repulsive_forces(int fd_npos,DroneMsg* drone_msg, double* force_x, double* force_y, double max_force,
    double M, double K, double T, int obstacles[N_OBS][2], int ro, double loop_prev_drone_pos[2],double loop_curr_drone_pos[2],
    double loop_next_drone_pos[2], int previous_drone_pos[2], int next_drone_pos[2], double* temp_x, double* temp_y){
    
    int distance = -1;
    // Computing ditances among drone and obstacles too see if in range
    for(int i = 0;i<N_OBS;i++){
        distance = sqrt(pow(drone_msg->new_drone_y - obstacles[i][0],2)+pow(drone_msg->new_drone_x - obstacles[i][1],2));
        if(distance <= ro){ // ro = 5
            // Drone is inside the obstacle repulsive area
            int obs_y_wrt_d = drone_msg->new_drone_y - obstacles[i][0];
            int obs_x_wrt_d = drone_msg->new_drone_x - obstacles[i][1];
            double angle = atan2(obs_y_wrt_d,obs_x_wrt_d);
            if(distance > (ro - ro*50/100)){
                // First area of repulsion
                *force_y = *force_y + (1)*sin(angle);
                *force_x = *force_x + (1)*cos(angle);
            }
            else if(distance > (ro - ro*75/100)){
                // Second area of repulsion
                *force_y = *force_y + max_force*sin(angle);
                *force_x = *force_x + max_force*cos(angle);
            }
            else {
                // Third area of repulsion
                *force_y = *force_y + (max_force*2)*sin(angle);
                *force_x = *force_x + (max_force*2)*cos(angle);
            }

            // Along x axis
            *temp_x = *force_x - (M/(T*T)*(loop_prev_drone_pos[1]-2*loop_curr_drone_pos[1])) + 
                    loop_curr_drone_pos[1]*(K/T);
            loop_next_drone_pos[1] = *temp_x/((M/(T*T))+(K/T));

            loop_prev_drone_pos[1] = loop_curr_drone_pos[1];
            loop_curr_drone_pos[1] = loop_next_drone_pos[1];

            next_drone_pos[1] = (int)round(loop_next_drone_pos[1]);

            previous_drone_pos[1] = drone_msg->new_drone_x;
            drone_msg->new_drone_x = next_drone_pos[1];

            // Along y axis
            *temp_y = *force_y - (M/(T*T)*(loop_prev_drone_pos[0]-2*loop_curr_drone_pos[0])) + 
                    loop_curr_drone_pos[0]*(K/T);
            loop_next_drone_pos[0] = *temp_y/((M/(T*T))+(K/T));

            loop_prev_drone_pos[0] = loop_curr_drone_pos[0];
            loop_curr_drone_pos[0] = loop_next_drone_pos[0];

            next_drone_pos[0] = (int)round(loop_next_drone_pos[0]);

            previous_drone_pos[0] = drone_msg->new_drone_y;
            drone_msg->new_drone_y = next_drone_pos[0];

            // New position is sent to blackboard for graphical update
            write(fd_npos,drone_msg,sizeof(*drone_msg));
        }
    }
}

void move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y, double max_force, 
       double oblique_force_comp, double M, double K, double T, int borders[], char last_valid_key, int obstacles[N_OBS][2], int ro){

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

        compute_repulsive_forces(fd_npos,drone_msg,&force_x,&force_y,max_force,M,K,T,obstacles,ro,
            loop_prev_drone_pos,loop_curr_drone_pos,loop_next_drone_pos, previous_drone_pos,next_drone_pos,
            &temp_x, &temp_y);

        // Checking if drone is within 5 pixels from the border
        // while(drone_msg->new_drone_y <= 6 || drone_msg->new_drone_x <= 6 || 
        while(drone_msg->new_drone_y <= 6 || drone_msg->new_drone_x <= 6 || 
            drone_msg->new_drone_y >= borders[0] || drone_msg->new_drone_x >= borders[1]){
            printf("border\n");
            is_on_border = 1;
            control = 0;
            delay = 150;

            // ----------

            float threshold = 6.0f;   // Distance from border where repulsion begins
            float k = max_force;      // Strength scale (you already have max_force)

            float fx = 0.0f;
            float fy = 0.0f;
            char artificial_key = 'n'; // n = neutral  

            // Distance from borders:
            float d_left   = drone_msg->new_drone_x - 6;
            float d_right  = borders[1] - drone_msg->new_drone_x;
            float d_top    = drone_msg->new_drone_y - 6;
            float d_bottom = borders[0] - drone_msg->new_drone_y;

            printf("d_left: %f, d_right: %f, d_top: %f, d_bottom: %f\n", d_left,d_right,d_top,d_bottom);

            // --- Horizontal repulsion ---
            if (d_left < threshold) {
                fx += k * (threshold - d_left) / threshold;  // push to +x
                artificial_key = 'f';
            }
            if (d_right < threshold) {
                fx -= k * (threshold - d_right) / threshold; // push to -x
                artificial_key = 's';
            }

            // --- Vertical repulsion ---
            if (d_top < threshold) {
                fy += k * (threshold - d_top) / threshold;   // push to +y
                artificial_key = 'c';
            }
            if (d_bottom < threshold) {
                fy -= k * (threshold - d_bottom) / threshold;// push to -y
                artificial_key = 'e';
            }

            printf("force x: %f, force y: %f\n", fx,fy);

            // Output force
            force_x = fx;
            force_y = fy;
            
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

            printf("force x: %f, force y: %f\n", force_x,force_y);

            // New position is sent to blackboard for graphical update
            write(fd_npos,drone_msg,sizeof(*drone_msg));

            //compute_repulsive_forces(fd_npos,drone_msg,&force_x,&force_y,max_force,M,K,T,obstacles,ro,loop_prev_drone_pos,
            //    loop_curr_drone_pos,loop_next_drone_pos, previous_drone_pos,next_drone_pos,&temp_x, &temp_y);

            sleep_ms(delay);

            control++;

            if(force_x > 0)
                force_x -= 1;
            else if(force_x < 0)
                force_x += 1;
            if(force_y > 0)
                force_y -= 1;
            else if(force_y < 0)
                force_y += 1;
        }
        //if(fabs(force_x) > epsilon || fabs(force_y) > epsilon){
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

            //compute_repulsive_forces(fd_npos,drone_msg,&force_x,&force_y,max_force,M,K,T,obstacles,ro,loop_prev_drone_pos,
            //    loop_curr_drone_pos,loop_next_drone_pos, previous_drone_pos,next_drone_pos,&temp_x, &temp_y);

            sleep_ms(delay);

            control++;

            if(force_x > 0)
                force_x -= 1;
            else if(force_x < 0)
                force_x += 1;
            if(force_y > 0)
                force_y -= 1;
            else if(force_y < 0)
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

    if(argc < 5){
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
    int obstacles[N_OBS][2];
    double force_on_x = 0.0;
    double force_on_y = 0.0;
    double max_applied_force;
    double M;
    double K;
    double T;
    int ro;

    load_parameters("parameters.txt",&M,&K,&T,&ro,&max_applied_force);

    double oblique_force_comp = (sqrt(2)/2)*max_applied_force;

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
                for(int i = 0;i<N_OBS;i++){
                    obstacles[i][0] = blackboard_msg.obstacles[i][0];
                    obstacles[i][1] = blackboard_msg.obstacles[i][1];
                }
                drone_msg.new_drone_y = blackboard_msg.drone_y;
                drone_msg.new_drone_x = blackboard_msg.drone_x;
                borders[0] = blackboard_msg.border_y;
                borders[1] = blackboard_msg.border_x;
                // Since it's at rest
                switch (received_key)
                {
                    // Updating drone position using Euler's method
                case 'f': force_on_x = max_applied_force; force_on_y = 0.0;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'f',obstacles, ro);
                    // Drone goes to the left (x changes)
                    break;
                case 's': force_on_x = -max_applied_force; force_on_y = 0.0;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'s',obstacles, ro);
                    // Drone goes to the left (x changes)
                    break;
                case 'e': force_on_x = 0.0; force_on_y = -max_applied_force;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'e',obstacles, ro);
                    // Drone goes up (y changes)
                    break;
                case 'c': force_on_x = 0.0; force_on_y = max_applied_force;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'c',obstacles, ro);
                    // Drone goes down (y changes)
                    break;
                case 'w': force_on_x = -oblique_force_comp; force_on_y = -oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'w',obstacles, ro);
                    // Drone goes to left/up
                    break;
                case 'r': force_on_x = oblique_force_comp; force_on_y = -oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'r',obstacles, ro);
                    // Drone goes right/up
                    break;
                case 'x': force_on_x = -oblique_force_comp; force_on_y = oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'x',obstacles, ro);
                    // Drone goes left/down
                    break;
                case 'v': force_on_x = oblique_force_comp; force_on_y = oblique_force_comp;
                        move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,'v',obstacles, ro);
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