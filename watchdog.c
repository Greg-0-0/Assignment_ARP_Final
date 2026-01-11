#include"functions.h"

int main(int argc, char* argv[]){
    if(argc < 11){
        fprintf(stderr,"No arguments passed to watchdog\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0);
    if (log_sem == SEM_FAILED) {
        perror("WATCHDOG line-11sem_open");
        exit(EXIT_FAILURE);
    }

    // Process identification logging
    pid_t pid = getpid();
    write_process_pid("processes.log", "WATCHDOG", pid, log_sem);

    int fd_from_b = atoi(argv[1]); // reads signal from blackboard
    int fd_from_d = atoi(argv[2]); // reads signal from drone
    int fd_from_i = atoi(argv[3]); // reads signal from input_manager
    int fd_from_o = atoi(argv[4]); // reads signal from obstacles
    int fd_from_t = atoi(argv[5]); // reads signal from targets
    pid_t b_pid = atoi(argv[6]); // blackboard PID
    pid_t d_pid = atoi(argv[7]); // drone PID
    pid_t i_pid = atoi(argv[8]); // input_manager PID
    pid_t o_pid = atoi(argv[9]); // obstacles PID
    pid_t t_pid = atoi(argv[10]); // targets PID

    const char *names[5] = {"BLACKBOARD", "DRONE", "INPUT_MANAGER", "OBSTACLES", "TARGETS"};
    int fds[5] = {fd_from_b, fd_from_d, fd_from_i, fd_from_o, fd_from_t}; // Array of fds to monitor
    pid_t pids[5] = {b_pid, d_pid, i_pid, o_pid, t_pid};
    time_t last_seen[5];
    int alerted[5] = {0,0,0,0,0}; // To keep track of alerted processes (timed out)
    for(int i = 0; i < 5; ++i) last_seen[i] = time(NULL);

    write_log("watchdog.log", "WATCHDOG", "INFO", "Watchdog started", log_sem);

    while(1){
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = -1;
        for(int i = 0; i < 5; ++i){
            FD_SET(fds[i], &rfds);
            if(fds[i] > maxfd) maxfd = fds[i];
        }

        // Set timeout for select
        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 500000; // 500 ms
        int ret = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if(ret == -1){
            // EINTR means the call was interrupted by a signal
            // This is normal and we should just retry the select
            // (since select is one of those functions that doesn't automatically restart even with SA_RESTART set)
            if(errno == EINTR){
                continue; // Retry select
            }
            // For other errors, log and exit
            log_error("watchdog.log", "WATCHDOG", "select", log_sem);
            perror("WATCHDOG line-70 select");
            exit(EXIT_FAILURE);
        }
        if(ret > 0){
            for(int i = 0; i < 5; ++i){
                // Check which fd is ready
                if(FD_ISSET(fds[i], &rfds)){
                    // Read up to 64 bytes to handle both pid_t heartbeats and "quit" string
                    char buf[64];
                    ssize_t n = read(fds[i], buf, sizeof(buf));
                    if(n > 0){
                        // Check for quit message (blackboard writes a 64-byte buffer with "quit" prefix)
                        if(n >= 4 && memcmp(buf, "quit", 4) == 0){
                            char msg[128];
                            snprintf(msg, sizeof msg, "Quit command received from %s", names[i]);
                            write_log("watchdog.log", "WATCHDOG", "INFO", msg, log_sem);

                            printf("Watchdog quitting as per command from %s.\n", names[i]);
                            for(int j = 0; j < 5; ++j) close(fds[j]);
                            sem_close(log_sem);
                            exit(EXIT_SUCCESS);
                        }

                        // Otherwise, treat pid_t heartbeats; multiple heartbeats may arrive batched
                        if(n >= (ssize_t)sizeof(pid_t)){
                            int count = n / (int)sizeof(pid_t);
                            for(int k = 0; k < count; ++k){
                                pid_t incoming;
                                memcpy(&incoming, buf + k * sizeof(pid_t), sizeof(incoming));
                                last_seen[i] = time(NULL); // Update last seen time
                                if(alerted[i]){
                                    // Process has recovered from timeout
                                    alerted[i] = 0;
                                    char msg[128];
                                    snprintf(msg, sizeof msg, "%s recovered", names[i]);
                                    write_log("watchdog.log", "WATCHDOG", "INFO", msg, log_sem);
                                    printf("%s (pid %d) has recovered and is sending heartbeats.\n", names[i], (int)incoming);
                                }
                                else{
                                    // Normal heartbeat received
                                    char msg[128];
                                    snprintf(msg, sizeof msg, "Heartbeat received from %s (pid %d)", names[i], (int)incoming);
                                    write_log("watchdog.log", "WATCHDOG", "INFO", msg, log_sem);
                                    printf("Heartbeat received from %s (pid %d).\n", names[i], (int)incoming);
                                }
                            }
                            // Any trailing bytes (n % sizeof(pid_t)) are ignored
                        }
                    }
                    else if(n == 0){
                        char msg[128];
                        snprintf(msg, sizeof msg, "%s pipe closed", names[i]);
                        write_log("watchdog.log", "WATCHDOG", "WARNING", msg, log_sem);
                    }
                    else{
                        log_error("watchdog.log", "WATCHDOG", "read heartbeat", log_sem);
                    }
                }
            }
        }

        time_t now = time(NULL);
        for(int i = 0; i < 5; ++i){
            if(!alerted[i] && difftime(now, last_seen[i]) > 3.0){
                // Process did not send heartbeat for more than 3 seconds -> alert
                char msg[160];
                snprintf(msg, sizeof msg, "%s (pid %d) timed out", names[i], (int)pids[i]);
                write_log("watchdog.log", "WATCHDOG", "ERROR", msg, log_sem);
                alerted[i] = 1;

                // Writing alert log
                printf("ALERT: %s (pid %d) has not sent heartbeat for over 3 seconds.\n", names[i], (int)pids[i]);
            }
        }
    }

    for(int i = 0; i < 5; ++i) close(fds[i]);
    sem_close(log_sem);
    return 0;
}