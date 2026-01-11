#include"functions.h"

int main(int argc, char * argv[]){

    if(argc < 4){
        fprintf(stderr,"No arguments passed to obstacles\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0);// Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("OBSTACLES line-12 sem_open");
        exit(EXIT_FAILURE);
    }

    // Process identification logging
    pid_t pid = getpid();
    write_process_pid("processes.log", "OBSTACLES", pid, log_sem);

    //Process started successfully
    write_log("application.log", "OBSTACLES", "INFO", "Obstacles process started successfully", log_sem);

    int fd_new_pos = atoi(argv[1]); // Receives position of drone and borders from blackboard
    int fd_new_obs = atoi(argv[2]); // Sends positions of obstacles
    int fd_hb_watchdog = atoi(argv[3]); // Heartbeat pipe to watchdog

    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0; 
    positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_NAN;
    
    srand(time(NULL));

    // Heartbeat using SIGALRM + ITIMER (sigaction with SA_RESTART inside)
    setup_heartbeat_itimer(1);

    // Make fd_new_pos non-blocking to allow heartbeat sending while waiting for data
    int pos_flags = fcntl(fd_new_pos, F_GETFL, 0);
    if(pos_flags < 0){
        perror("OBSTACLES line-38 fcntl F_GETFL");
        exit(EXIT_FAILURE);
    }
    pos_flags |= O_NONBLOCK;
    if(fcntl(fd_new_pos, F_SETFL, pos_flags) < 0){
        perror("OBSTACLES line-43 fcntl F_SETFL");
        exit(EXIT_FAILURE);
    }

    while(1){

        // Heartbeat if due
        send_heartbeat_if_due(fd_hb_watchdog, "OBSTACLES", log_sem);

        ssize_t n = read_full(fd_new_pos,&positions,sizeof(positions));
        
        // Heartbeat after blocking read
        send_heartbeat_if_due(fd_hb_watchdog, "OBSTACLES", log_sem);
        // The call may read bytes not belonging to the same struct (BlackboardMsg), due to race conditions on pipe creating junk inside the pipe,
        // or sending different struct on the same pipe.
        // Just to be clear my code doesn't seem to cause these race conditions, neither it uses the same file descriptors to send different type of structures,
        // however, this did happen, and, as it took place, it vanished once I manually checked the number of bytes read and expected:
        // printf("Read bytes: %ld, expected: %ld", n, sizeof(positions));
        // which of course is quite odd, other than frustrating, since it shouldn't have fixed the issue.
        // This happens because the error itself is an event, I would say, completely unpredictable, that depends on the integrity of pipes themself,
        // which, unfortunatley, I am forced to use.
        
        if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)){
            // No data available, sleep briefly to avoid busy-waiting
            sleep_ms(50); // 50ms
            continue;
        }
        
        if(n > 0){
            if(positions.type == MSG_QUIT){
                write_log("application.log", "OBSTACLES", "INFO", "Obstacles process terminated successfully", log_sem);
                exit(EXIT_SUCCESS);
            }
            else if(positions.type == MSG_NOB){
                // Changing obstacle position
                for(int i = 0;i<N_OBS;i++){
                    int pos_y = 7 + rand() % (positions.border_y - 7); // Random generator from 7 to H - 8
                    int pos_x = 7 + rand() % (positions.border_x - 7); // Random generator from 7 to W - 8

                    // Checking if new position collides with targets, drone or other obstacles
                    if(check_position(pos_y, pos_x, positions, i-1, 0, 1)){
                        i--;
                        continue;
                    }
                    positions.obstacles[i][0] = pos_y;
                    positions.obstacles[i][1] = pos_x;
                }
            }
            else{
                // Creating obstacles
                for(int i = 0;i<N_OBS;i++){
                    int pos_y = 7 + rand() % (positions.border_y - 7); // Random generator from 7 to H - 8
                    int pos_x = 7 + rand() % (positions.border_x - 7); // Random generator from 7 to W - 8
                    // Checking if new position collides with drone or other obstacles
                    if(check_position(pos_y, pos_x, positions, i-1, 0, 0)){
                        i--;
                        continue;
                    }
                    positions.obstacles[i][0] = pos_y;
                    positions.obstacles[i][1] = pos_x;
                }

            }
            write(fd_new_obs,&positions,sizeof(positions));
            
            // Heartbeat after processing and writing
            send_heartbeat_if_due(fd_hb_watchdog, "OBSTACLES", log_sem);
        }
        else{
            log_error("application.log", "OBSTACLES", "read obstacles", log_sem);
            perror("OBSTACLES line-113 read obstacles");
            exit(EXIT_FAILURE);
        }


    }
    close(fd_new_pos);
    close(fd_new_obs);

    write_log("application.log", "OBSTACLES", "INFO", "Obstacles process terminated successfully", log_sem);
    
    sem_close(log_sem);

    return 0;
}