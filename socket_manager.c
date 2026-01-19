#include"functions.h"

int main(int argc, char* argv[]) {

    if(argc < 8){
        fprintf(stderr,"Not enough arguments passed to socket manager\n");
        exit(EXIT_FAILURE);
    }

    sem_t *log_sem = sem_open("/log_sem", 0); // Open existing semaphore for logging
    if (log_sem == SEM_FAILED) {
        perror("SOCKET_MANAGER sem_open");
        exit(EXIT_FAILURE);
    }

    // Process started successfully
    write_log("application.log", "SOCKET_MANAGER", "INFO", "Socket Manager process started successfully", log_sem);

    // Receiving reads are blocking
    int fd_to_bb = atoi(argv[1]); // If server: writes client drone position to blackboardserver (seen as obstacle)
                                  // If client: writes window size to blackboardclient
    // Receiving reads are non-blocking
    int fd_to_bb_2 = atoi(argv[2]); // Only client: writes termination/server drone position/request for obs pos to blackboardclient
    int fd_from_bb = atoi(argv[3]); // Reads obstacle position or window size if client (even if server -> not sure yet) from blackboard (non-blocking)
    int server_client_flag = atoi(argv[4]); // Flag indicating blackboard mode (1 -> server, 2 -> client)
    int port_number = atoi(argv[5]); // Port number for socket communication
    char* server_ip = argv[6]; // Server IP address (only for client)
    int fd_from_bb_2 = atoi(argv[7]); // Reads from blackboardclient to exit correctly (only for client)
    BlackboardMsg positions; positions.type = MSG_NAN;
    int wind_H = 0; // Used to go from local coordinate system (origin in top_left corner) to virtual coordinate system (origin in bottom_left corner) and vice versa
    int first_time = 1; // Used by server to give the user the possibility to either connect or terminate

    if(server_client_flag == 1){
        // Server mode

        // Non-blocking write to blackboard (used only to send new obstacle position)
        int to_bb_flags = fcntl(fd_to_bb, F_GETFL, 0);
        if(to_bb_flags >= 0){
            to_bb_flags |= O_NONBLOCK;
            (void)fcntl(fd_to_bb, F_SETFL, to_bb_flags);
        }

        // Defining variables for socket programming
        int sockfd, newsockfd, portno, clilen;
        char buffer_input[256], buffer_output[256];
        struct sockaddr_in serv_addr, cli_addr;

        // Creating socket
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) { 
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error opening socket", log_sem);
            perror("SOCKET_MANAGER opening socket");
            exit(EXIT_FAILURE);
        }
        
        // Enable socket address reuse to avoid "Address already in use" errors
        int opt = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error setting socket options", log_sem);
            perror("SOCKET_MANAGER setsockopt");
            exit(EXIT_FAILURE);
        }
        
        memset(&serv_addr, 0, sizeof(serv_addr));
        portno = port_number;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY; // Address of the machine
        serv_addr.sin_port = htons(portno); // Port number conversion to network byte order

        // Binding socket to address structure
        if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error on binding", log_sem);
            perror("SOCKET_MANAGER binding");
            exit(EXIT_FAILURE);
        }

        // Listening for connection
        listen(sockfd,5);
        clilen = sizeof(cli_addr);

        // Main server loop to accept connections
        while(1){
            // Ready to accept connection from client
            if(!first_time){
                printf("SOCKET_MANAGER: Accept new client on port %d(y) or exit(n)\n", portno);
                char choice = getchar();
                if(choice == 'n' || choice == 'N'){
                    write_log("application.log", "SOCKET_MANAGER", "INFO", "Socket Manager process terminated by user", log_sem);

                    positions.type = MSG_QUIT;
                    write(fd_to_bb, &positions, sizeof(positions)); // Notify blackboard about termination

                    // Small delay to ensure quit message is flushed before exiting
                    sleep(1);

                    close(sockfd);
                    exit(EXIT_SUCCESS);
                }
                else if(choice != 'y' && choice != 'Y'){
                    printf("Error: Invalid choice, exiting...\n");
                    close(sockfd);
                    exit(EXIT_FAILURE);
                }
            }
            else{
                first_time = 0;
            }
            printf("SOCKET_MANAGER: Waiting for client connection on port %d...\n", portno);
            newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
            if (newsockfd < 0) {
                write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error on accept", log_sem);
                perror("SOCKET_MANAGER accept");
                close(sockfd);
                exit(EXIT_FAILURE);
            }

            // Disable Nagle's algorithm to prevent message coalescing
            int flag = 1;
            if (setsockopt(newsockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
                write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error setting TCP_NODELAY", log_sem);
                perror("SOCKET_MANAGER setsockopt TCP_NODELAY");
            }

            // Send first message to client to confirm connection
            memset(buffer_output, 0, 256);
            snprintf(buffer_output, 256, "ok\n");
            int n = write(newsockfd, buffer_output, strlen(buffer_output));
            if (n < 0) {
                perror("SOCKET_MANAGER writing to socket");
                error(newsockfd, sockfd, "Error writing to socket", log_sem);
            }

            // Wait for acknowledgment from client
            int ook_ret = read_line(newsockfd, buffer_input, sizeof(buffer_input));
            if(ook_ret == 0) {
                write_log("application.log", "SOCKET_MANAGER", "INFO", "Client disconnected after window size", log_sem);
                close(newsockfd);
                continue; // Return to accept() for new connection
            }
            if(ook_ret < 0) {
                write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error line 136 reading acknowledgment from client", log_sem);
                close(newsockfd);
                close(sockfd);
                exit(EXIT_FAILURE);
            }
            if(strncmp(buffer_input, "ook", 3) != 0)
                error(newsockfd, sockfd, "Invalid acknowledgment from client", log_sem);

            // Connection established with client
            write_log("application.log", "SOCKET_MANAGER", "INFO", "Connection established with client", log_sem);

            // Send window size
            read(fd_from_bb, &positions, sizeof(positions)); // Retrieving window size from blackboard
            memset(buffer_output, 0, 256);
            analyze_position_n_size_and_prepare_message(positions, buffer_output, -1); // wind_H not needed here
            n = write(newsockfd, buffer_output, strlen(buffer_output));
            if (n < 0) {
                perror("SOCKET_MANAGER writing window size to socket");
                error(newsockfd, sockfd, "Error writing window size to socket", log_sem);
            }

            // Store window height for coordinate conversions
            wind_H = positions.border_y + 7;

            // Wait for acknowledgment from client
            memset(buffer_input, 0, 256);
            int sok_ret = read_line(newsockfd, buffer_input, 255);
            if(sok_ret == 0) {
                write_log("application.log", "SOCKET_MANAGER", "INFO", "Client disconnected after window size", log_sem);
                close(newsockfd);
                continue; // Return to accept() for new connection
            }
            if(sok_ret < 0) {
                write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error line 169 reading acknowledgment from client", log_sem);
                close(newsockfd);
                close(sockfd);
                exit(EXIT_FAILURE);
            }
            if(strncmp(buffer_input, "sok", 3) != 0)
                error(newsockfd, sockfd, "Invalid acknowledgment for window size from client", log_sem);

            // Communication loop with client (reads/writes)
            while(1){

                // Reads from blackboard to either quit or get new drone position
                read(fd_from_bb, &positions, sizeof(positions));
                if(positions.type == MSG_QUIT){
                    // Quitting socket manager

                    // Notify client about termination
                    snprintf(buffer_output, 256, "q\n");
                    write(newsockfd, buffer_output, strlen(buffer_output));

                    // Wait for acknowledgment from client
                    memset(buffer_input, 0, 256);
                    int qok_ret = read_line(newsockfd, buffer_input, sizeof(buffer_input));
                    if(qok_ret == 0) {
                        write_log("application.log", "SOCKET_MANAGER", "INFO", "Client disconnected after window size", log_sem);
                        close(newsockfd);
                        break; // Return to accept() for new connection
                    }
                    if(qok_ret < 0) {
                        write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error line 198 reading acknowledgment from client", log_sem);
                        close(sockfd);
                        close(newsockfd);
                        exit(EXIT_FAILURE);
                    }
                    if(strncmp(buffer_input, "qok", 3) != 0)
                        error(newsockfd, sockfd, "Invalid acknowledgment for quit from client", log_sem);

                    write_log("application.log", "SOCKET_MANAGER", "INFO", "Socket Manager process terminated successfully", log_sem);
                    close(newsockfd);
                    close(sockfd);
                    exit(EXIT_SUCCESS);
                }

                // Notify client about new position available
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "drone\n");
                n = write(newsockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER writing drone position to socket");
                    error(newsockfd, sockfd, "Error writing drone position to socket", log_sem);
                }

                // Send new drone position
                memset(buffer_output, 0, 256);
                analyze_position_n_size_and_prepare_message(positions, buffer_output, wind_H);

                n = write(newsockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER writing drone position to socket");
                    error(newsockfd, sockfd, "Error writing drone position to socket", log_sem);
                }

                // Wait for acknowledgment from client
                memset(buffer_input, 0, 256);
                int dok_ret = read_line(newsockfd, buffer_input, sizeof(buffer_input));
                if(dok_ret == 0) {
                    write_log("application.log", "SOCKET_MANAGER", "INFO", "Client disconnected after drone position", log_sem);
                    close(newsockfd);
                    break; // Exit communication loop
                }
                if(dok_ret < 0) {
                    write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error line 240 reading drone position acknowledgment", log_sem);
                    close(newsockfd);
                    close(sockfd);
                    exit(EXIT_FAILURE);
                }
                if(strncmp(buffer_input, "dok", 3) != 0)
                    error(newsockfd, sockfd, "Invalid acknowledgment for drone position from client", log_sem);

                // Request for obstacle position from client
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "obst\n");
                n = write(newsockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER requesting obstacle position from socket");
                    error(newsockfd, sockfd, "Error requesting obstacle position from socket", log_sem);
                }

                // Wait for obstacle position from client
                memset(buffer_input, 0, 256);
                int obst_pos = read_line(newsockfd, buffer_input, sizeof(buffer_input));
                if(obst_pos == 0) {
                    write_log("application.log", "SOCKET_MANAGER", "INFO", "Client disconnected while sending obstacle position", log_sem);
                    close(newsockfd);
                    break; // Exit communication loop
                }
                if(obst_pos < 0) {
                    write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error line 266 reading obstacle position from client", log_sem);
                    close(newsockfd);
                    close(sockfd);
                    exit(EXIT_FAILURE);
                }
                int obs_x = 0, obs_y = 0;
                sscanf(buffer_input, "%d, %d", &obs_x, &obs_y); // x, y on the wire
                positions.drone_x = obs_x;
                positions.drone_y = wind_H - obs_y; // Convert back from bottom-left to top-left origin
                positions.type = MSG_NPOS; // Signaling blackboard that this is an obstacle position
                write(fd_to_bb, &positions, sizeof(positions)); // Sending obstacle position to blackboard // DEBUG: doesn't fire

                // Send acknowledgment to client
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "pok\n");
                n = write(newsockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER sending acknowledgment to socket");
                    error(newsockfd, sockfd, "Error sending acknowledgment to socket", log_sem);
                }
            }
        }
        
        close(sockfd);
    }
    else{
        // Client mode

        // Non-blocking read from blackboard (used to detect client shutdown without stalling protocol)
        int from_bb2_flags = fcntl(fd_from_bb_2, F_GETFL, 0);
        if(from_bb2_flags >= 0){
            from_bb2_flags |= O_NONBLOCK;
            (void)fcntl(fd_from_bb_2, F_SETFL, from_bb2_flags);
        }
        else{
            perror("fcntl F_GETFL for socket manager client");
        }

        // Defining variables for socket programming
        int sockfd, portno, n;
        struct sockaddr_in serv_addr;

        char buffer_input[256], buffer_output[256];

        // Creating socket
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) { 
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error opening socket", log_sem);
            perror("SOCKET_MANAGER opening socket");
            exit(EXIT_FAILURE);
        }

        // Disable Nagle's algorithm to prevent message coalescing
        int flag = 1;
        if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error setting TCP_NODELAY", log_sem);
            perror("SOCKET_MANAGER setsockopt TCP_NODELAY");
        }

        portno = port_number;
        
        // Resolve using gethostbyname() (IPv4)
        struct hostent *server;
        // Trim trailing newlines/spaces from server_ip to avoid resolution issues
        size_t len = strlen(server_ip);
        while (len > 0 && (server_ip[len-1] == '\n' || server_ip[len-1] == '\r' || server_ip[len-1] == ' ')) {
            server_ip[len-1] = '\0';
            len--;
        }

        server = gethostbyname(server_ip);
        if (server == NULL) {
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "gethostbyname failed: Name or service not known", log_sem);
            fprintf(stderr, "SOCKET_MANAGER: gethostbyname failed (server_ip=%s, port=%d)\n", server_ip, port_number);
            exit(EXIT_FAILURE);
        }

        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        memcpy(&serv_addr.sin_addr.s_addr, server->h_addr_list[0], server->h_length);
        serv_addr.sin_port = htons(portno);

        // Connecting to server
        if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
            write_log("application.log", "SOCKET_MANAGER", "ERROR", "Error connecting to server", log_sem);
            perror("SOCKET_MANAGER connecting to server");
            exit(EXIT_FAILURE);
        }

        // Wait for initial message from server
        memset(buffer_input, 0, 256);
        if(read_line(sockfd, buffer_input, 255) <= 0) {
            error(-1, sockfd, "Error line 267 reading from socket", log_sem);
            exit(EXIT_FAILURE);
        }
        if(strncmp(buffer_input, "ok", 2) != 0)
            error(-1, sockfd, "Invalid initial message from server", log_sem);
        
        // Send acknowledgment to server
        memset(buffer_output, 0, 256);
        snprintf(buffer_output, 256, "ook\n");
        n = write(sockfd, buffer_output, strlen(buffer_output));
        if (n < 0) {
            perror("SOCKET_MANAGER writing acknowledgment to socket");
            error(-1, sockfd, "Error writing acknowledgment to socket", log_sem);
        }

        // Connection established with server
        write_log("application.log", "SOCKET_MANAGER", "INFO", "Connection established with server", log_sem);

        // Receive window size from server
        memset(buffer_input, 0, 256);
        if(read_line(sockfd, buffer_input, 255) <= 0) {
            error(-1, sockfd, "Error line 288 reading from socket", log_sem);
            exit(EXIT_FAILURE);
        }
        if(strncmp(buffer_input, "size", 4) != 0)
            error(-1, sockfd, "Invalid window size message from server", log_sem);

        // Send acknowledgment to server
        memset(buffer_output, 0, 256);
        snprintf(buffer_output, 256, "sok\n");
        n = write(sockfd, buffer_output, strlen(buffer_output));
        if (n < 0) {
            perror("SOCKET_MANAGER writing acknowledgment to socket");
            error(-1, sockfd, "Error writing acknowledgment to socket", log_sem);
        }

        // Parsing window size (width, height)
        int win_w = 0, win_h = 0;
        sscanf(buffer_input, "size %d, %d", &win_w, &win_h);
        positions.type = MSG_WSIZE;
        positions.border_x = win_w; // Width (cols)
        positions.border_y = win_h; // Height (rows)
        wind_H = win_h; // Storing window height for coordinate conversions
        write(fd_to_bb, &positions, sizeof(positions)); // Sending window size to blackboard

        // Communication loop with server (reads/writes)
        while(1){
            // Wait for message from server
            memset(buffer_input, 0, 256);
            if(read_line(sockfd, buffer_input, sizeof(buffer_input)) <= 0) {
                error(-1, sockfd, "Error line 317 reading from socket", log_sem);
                exit(EXIT_FAILURE);
            }
            if(strncmp(buffer_input, "q", 1) == 0){
                // Quitting socket manager

                // Send acknowledgment to server
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "qok\n");
                n = write(sockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER writing acknowledgment to socket");
                    error(-1, sockfd, "Error writing acknowledgment to socket", log_sem);
                }

                // Notify blackboard about termination
                positions.type = MSG_QUIT;
                write(fd_to_bb_2, &positions, sizeof(positions));

                write_log("application.log", "SOCKET_MANAGER", "INFO", "Socket Manager process terminated successfully", log_sem);
                close(sockfd);
                exit(EXIT_SUCCESS);
            }
            else if(strncmp(buffer_input, "drone", 5) == 0){
                // Receiving new drone position from server

                // Read drone position
                memset(buffer_input, 0, 256);

                if(read_line(sockfd, buffer_input, 255) <= 0) {
                    error(-1, sockfd, "Error line 347 reading from socket", log_sem);
                    exit(EXIT_FAILURE);
                }

                int srv_x = 0, srv_y = 0;
                sscanf(buffer_input, "%d, %d", &srv_x, &srv_y); // x, y on the wire
                positions.drone_x = srv_x;
                positions.drone_y = wind_H - srv_y; // Convert back from bottom-left to top-left origin
                positions.type = MSG_NPOS; // Signaling blackboard that this is drone position

                write(fd_to_bb_2, &positions, sizeof(positions)); // Sending drone position to blackboard

                // Send acknowledgment to server
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "dok\n");
                n = write(sockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER writing acknowledgment to socket");
                    error(-1, sockfd, "Error writing acknowledgment to socket", log_sem);
                }
            }
            else if(strncmp(buffer_input, "obst", 4) == 0){
                // Sending drone position to server (seen as obstacle)

                // Request drone position from blackboard
                positions.type = MSG_NOB; // Signaling blackboard to send obstacle position
                write(fd_to_bb_2, &positions, sizeof(positions));

                // Read obstacle position from blackboard
                read(fd_from_bb, &positions, sizeof(positions));

                // Convert to virtual coordinates expected on the wire (x, y)
                const int obs_x = positions.drone_x;
                const int obs_y_virtual = wind_H - positions.drone_y;

                // Send obstacle position to server
                memset(buffer_output, 0, 256);
                snprintf(buffer_output, 256, "%d, %d\n", obs_x, obs_y_virtual);
                n = write(sockfd, buffer_output, strlen(buffer_output));
                if (n < 0) {
                    perror("SOCKET_MANAGER writing obstacle position to socket");
                    error(-1, sockfd, "Error writing obstacle position to socket", log_sem);
                }

                // Wait for acknowledgment from server
                memset(buffer_input, 0, 256);
                if(read_line(sockfd, buffer_input, 255) <= 0) {
                    error(-1, sockfd, "Error line 389 reading from socket", log_sem);
                    exit(EXIT_FAILURE);
                }
                if(strncmp(buffer_input, "pok", 3) != 0)
                    error(-1, sockfd, "Invalid acknowledgment for obstacle position from server", log_sem);
            }

            
            // Cheking if client application exited (close socket and exit)
            // Non-blocking read from blackboard; ignore EAGAIN/EWOULDBLOCK to avoid stalling protocol
            ssize_t rb = read(fd_from_bb_2, &positions, sizeof(positions));
            if(rb == sizeof(positions)){
                if(positions.type == MSG_QUIT){
                    // Quitting socket manager
                    write_log("application.log", "SOCKET_MANAGER", "INFO", "Socket Manager process terminated successfully", log_sem);
                    close(sockfd);
                    exit(EXIT_SUCCESS);
                }
            }
            else if(rb == -1 && errno != EAGAIN && errno != EWOULDBLOCK){
                perror("SOCKET_MANAGER reading quit from blackboard");
            }
            
        }
        close(sockfd);
    }
    close(fd_to_bb);
    close(fd_to_bb_2);
    close(fd_from_bb);

    sem_close(log_sem);
    sem_unlink("/log_sem");

    return 0;
}