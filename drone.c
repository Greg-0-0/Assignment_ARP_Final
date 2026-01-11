// gcc drone.c -o drone -lm
// Correct order of execution: konsole -e ./blackboard& -> konsole -e ./drone& -> konsole -e ./input_manager&

#include"functions.h"

int main(int argc, char* argv[]){

    if(argc < 7){
        fprintf(stderr,"No arguments passed to drone\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0);// Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("DRONE sem_open");
        exit(EXIT_FAILURE);
    }

    // Process identification logging
    pid_t pid = getpid();
    write_process_pid("processes.log", "DRONE", pid, log_sem);

    // Process started successfully
    write_log("application.log", "DRONE", "INFO", "Drone process started successfully", log_sem);

    // Creation of file descriptors
    int fd_req = atoi(argv[1]); // Writes to blackboard a request for obstacles, borders and current drone position
    int fd_npos = atoi(argv[2]); // Writes to blackboard new drone position after movement
    int fd_pos = atoi(argv[3]); // Reads obstcles, borders and drone position
    int fd_key = atoi(argv[4]); // Reads user key from input_manager
    int fd_hb_watchdog = atoi(argv[5]); // Heartbeat pipe to watchdog
    int application_flag = atoi(argv[6]); // Flag indicating standalone or networked mode

    if(application_flag)
        // Heartbeat using SIGALRM + ITIMER (sigaction with SA_RESTART inside)
        setup_heartbeat_itimer(1);

    // Make fd_key non-blocking to allow heartbeat sending while waiting for input
    int key_flags = fcntl(fd_key, F_GETFL, 0);
    if(key_flags < 0){
        perror("DRONE fcntl F_GETFL");
        exit(EXIT_FAILURE);
    }
    key_flags |= O_NONBLOCK;
    if(fcntl(fd_key, F_SETFL, key_flags) < 0){
        perror("DRONE fcntl F_SETFL");
        exit(EXIT_FAILURE);
    }

    char received_key;
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

    if(load_parameters("parameters.txt",&M,&K,&T,&ro,&max_applied_force) < 0){
        log_error("application.log", "DRONE", "load_parameters", log_sem);
        printf("Exiting process drone.\n");
        exit(EXIT_FAILURE);
    }

    double oblique_force_comp = (sqrt(2)/2)*max_applied_force;

    BlackboardMsg blackboard_msg;
    DroneMsg drone_msg; drone_msg.type = MSG_NAN;

    while(1){

        if(application_flag)
            // Heartbeat if due
            send_heartbeat_if_due(fd_hb_watchdog, "DRONE", log_sem);

        int n = read(fd_key,&received_key,1);
        
         if(application_flag)
            // Heartbeat after read
            send_heartbeat_if_due(fd_hb_watchdog, "DRONE", log_sem);
        
        if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)){
            // No data available, sleep briefly to avoid busy-waiting
            sleep_ms(50); // 50ms
            continue;
        }
        
        if(n > 0){
            if(received_key == 'q'){
                // Quitting programs
                drone_msg.type = MSG_QUIT;
                write(fd_req,&drone_msg,sizeof(drone_msg)); // Sent to blackboard
                write_log("application.log", "DRONE", "INFO", "Drone process terminated successfully", log_sem);
                exit(EXIT_SUCCESS);
            }
            if(received_key == 'f' || received_key == 's' || received_key == 'e' || received_key == 'c' || 
                received_key == 'w' || received_key == 'r' || received_key == 'x' || received_key == 'v' ||
                received_key == 'o'){
                 // Request current positions from blackboard
                drone_msg.type = MSG_POS;
                write(fd_req, &drone_msg, sizeof(drone_msg)); // Requests drone, obstacles and borders positions
                read(fd_pos, &blackboard_msg, sizeof(blackboard_msg)); // Receives positions
                for(int i = 0;i<N_OBS;i++){
                    obstacles[i][0] = blackboard_msg.obstacles[i][0];
                    obstacles[i][1] = blackboard_msg.obstacles[i][1];
                    // If newtworked mode all obstacles positions, except the first one, are (-1,-1)
                }
                drone_msg.new_drone_y = blackboard_msg.drone_y;
                drone_msg.new_drone_x = blackboard_msg.drone_x;
                borders[0] = blackboard_msg.border_y;
                borders[1] = blackboard_msg.border_x;
                
                int quit_requested = 0;
                switch (received_key)
                {
                    // Updating drone position using Euler's method
                case 'f': force_on_x = max_applied_force; force_on_y = 0.0;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes to the right (x changes)
                    break;
                case 's': force_on_x = -max_applied_force; force_on_y = 0.0;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes to the left (x changes)
                    break;
                case 'e': force_on_x = 0.0; force_on_y = -max_applied_force;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes up (y changes)
                    break;
                case 'c': force_on_x = 0.0; force_on_y = max_applied_force;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes down (y changes)
                    break;
                case 'w': force_on_x = -oblique_force_comp; force_on_y = -oblique_force_comp;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes to left/up
                    break;
                case 'r': force_on_x = oblique_force_comp; force_on_y = -oblique_force_comp;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes right/up
                    break;
                case 'x': force_on_x = -oblique_force_comp; force_on_y = oblique_force_comp;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes left/down
                    break;
                case 'v': force_on_x = oblique_force_comp; force_on_y = oblique_force_comp;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    // Drone goes right/down
                    break;
                case 'o': 
                        // Artificial key sent when client obstacle position updated, but drone didn't move
                        // Just check for collision without moving the drone
                        force_on_x = 0.0; force_on_y = 0.0;
                        ro = 7;
                        quit_requested = move_drone(fd_key,fd_npos,&drone_msg,next_drone_position,force_on_x,force_on_y,
                        max_applied_force + 1.0,oblique_force_comp,M,K,T,borders,obstacles, ro, log_sem);
                    break;
                case 'q':
                    drone_msg.type = MSG_QUIT;
                    write(fd_req,&drone_msg,sizeof(drone_msg));
                    write_log("application.log", "DRONE", "INFO", "Drone process terminated successfully", log_sem);
                    exit(EXIT_SUCCESS);
                default:
                    drone_msg.type = MSG_NAN;
                    write(fd_npos,&drone_msg,sizeof(drone_msg));
                    break;
                }
                
                // Check if user pressed 'q' during movement
                if(quit_requested){
                    drone_msg.type = MSG_QUIT;
                    write(fd_req,&drone_msg,sizeof(drone_msg));
                    write_log("application.log", "DRONE", "INFO", "Drone process terminated successfully", log_sem);
                    exit(EXIT_SUCCESS);
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

    sem_close(log_sem);

    return 0;
}