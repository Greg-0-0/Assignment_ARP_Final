#include"functions.h"

int main(int argc, char * argv[]){

    if(argc < 4){
        fprintf(stderr,"No arguments passed to targets\n");
        exit(EXIT_FAILURE);
    }
  
    sem_t *log_sem = sem_open("/log_sem", 0);// Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("TARGETS line-12 sem_open");
        exit(EXIT_FAILURE);
    }

    // Process identification logging
    pid_t pid = getpid();
    write_process_pid("processes.log", "TARGETS", pid, log_sem);

    //Process started successfully
    write_log("application.log", "TARGETS", "INFO", "Targets process started successfully", log_sem);

    int fd_new_pos = atoi(argv[1]); // Receives positions of drone and blackboard
    int fd_new_trs = atoi(argv[2]); // Sends position of targets
    int fd_hb_watchdog = atoi(argv[3]); // Heartbeat pipe to watchdog

    BlackboardMsg positions; positions.border_x = 0; positions.border_y = 0; 
    positions.drone_x = 0; positions.drone_y = 0; positions.type = MSG_NAN;
    
    srand(time(NULL));

    // Heartbeat using SIGALRM + ITIMER (sigaction with SA_RESTART inside)
    setup_heartbeat_itimer(1);

    // Make fd_new_pos non-blocking to allow heartbeat sending while waiting for data
    int pos_flags = fcntl(fd_new_pos, F_GETFL, 0);
    if(pos_flags < 0){
        perror("TARGETS line-38 fcntl F_GETFL");
        exit(EXIT_FAILURE);
    }
    pos_flags |= O_NONBLOCK;
    if(fcntl(fd_new_pos, F_SETFL, pos_flags) < 0){
        perror("TARGETS line-43 fcntl F_SETFL");
        exit(EXIT_FAILURE);
    }

    while(1){

        // Heartbeat if due
        send_heartbeat_if_due(fd_hb_watchdog, "TARGETS", log_sem);

        ssize_t n = read_full(fd_new_pos,&positions,sizeof(positions));
        
        // Heartbeat after blocking read
        send_heartbeat_if_due(fd_hb_watchdog, "TARGETS", log_sem);
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
                write_log("application.log", "TARGETS", "INFO", "Targets process terminated successfully", log_sem);
                exit(EXIT_SUCCESS);
            }
            // Creating targets
            for(int i = 0;i<N_TARGETS;i++){
                int pos_y = 7 + rand() % (positions.border_y - 7); // Random generator from 7 to H - 8
                int pos_x = 7 + rand() % (positions.border_x - 7); // Random generator from 7 to W - 8
                if(check_position(pos_y, pos_x, positions, i-1, 1, 0)){
                    i--;
                    continue;
                }
                positions.targets[i][0] = pos_y;
                positions.targets[i][1] = pos_x;
            }

            write(fd_new_trs,&positions,sizeof(positions));
            
            // Heartbeat after processing and writing
            send_heartbeat_if_due(fd_hb_watchdog, "TARGETS", log_sem);
        }
        else if(n == 0){
            // Pipe closed
            write_log("application.log", "TARGETS", "WARNING", "Pipe closed", log_sem);
            exit(EXIT_SUCCESS);
        }
        else if(n < 0){
            // Real error (not EAGAIN which was already handled)
            log_error("application.log", "TARGETS", "read targets", log_sem);
            perror("TARGETS line-101read targets");
            exit(EXIT_FAILURE);
        }


    }
    close(fd_new_pos);
    close(fd_new_trs);

    sem_close(log_sem);

    return 0;
}