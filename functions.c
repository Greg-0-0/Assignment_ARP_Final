#include"functions.h"

// Define the global variable (declared as extern in .h)
volatile sig_atomic_t update_obstacles = 0;
volatile sig_atomic_t heartbeat_due = 0;
static timer_t heartbeat_timer_id;

// ------ used in blackboard.c ------

void draw_rect(WINDOW *win, int y, int x, int h, int w, int color_pair)
{
    wattron(win, COLOR_PAIR(color_pair));

    // top and bottom
    for (int col = x; col < w; col++) {
        mvwaddch(win, y, col, ACS_HLINE);
        mvwaddch(win, h, col, ACS_HLINE);
    }

    // left and right
    for (int row = y; row < h; row++) {
        mvwaddch(win, row, x, ACS_VLINE);
        mvwaddch(win, row, w, ACS_VLINE);
    }

    // corners
    mvwaddch(win, y, x, ACS_ULCORNER);
    mvwaddch(win, y, w, ACS_URCORNER);
    mvwaddch(win, h, x, ACS_LLCORNER);
    mvwaddch(win, h, w, ACS_LRCORNER);

    wattroff(win, COLOR_PAIR(color_pair));

    wrefresh(win);
}

void layout_and_draw(WINDOW *win) {


    int H, W;
    getmaxyx(stdscr, H, W);
    start_color();

    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);

    // Windows with fixed margins
    int wh = (H > 6) ? H - 6 : H;
    int ww = (W > 10) ? W - 10 : W;
    if (wh < 3) wh = 3;
    if (ww < 3) ww = 3;

    // Resize and recenter the window
    wresize(win, wh, ww);
    mvwin(win, (H - wh) / 2, (W - ww) / 2);

    // Klean up and redraw
    werase(stdscr);
    werase(win);
    box(win, 0, 0);
    getmaxyx(win, H, W);
    // Drawable region goes from y:1 to H-2 and x:1 to W-2

    // Spawn drone
    wattron(win, COLOR_PAIR(4));
    mvwprintw(win,H/2,W/2,"+");
    wattroff(win, COLOR_PAIR(4));

    refresh();
    wrefresh(win);

    draw_rect(win,6,6,H-7,W-7,1);
}

void layout_and_draw_for_networked_app(WINDOW *win, int server_client_flag, int height, int width) {

    int H, W;
    if(server_client_flag){
        // Client: set fixed dimensions received from server
        H = height;
        W = width;
    }
    else
        // Server: get terminal dimensions
        getmaxyx(stdscr, H, W);

    start_color();

    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);

    int wh, ww;
    if(server_client_flag){
        // Client: set fixed dimensions received from server
        wh = height;
        ww = width;
    }
    else {
        // Windows with fixed margins
        wh = (H > 6) ? H - 6 : H;
        ww = (W > 10) ? W - 10 : W;
        if (wh < 3) wh = 3;
        if (ww < 3) ww = 3;
    }

    // Resize and recenter the window
    wresize(win, wh, ww);
    mvwin(win, (H - wh) / 2, (W - ww) / 2);

    // Klean up and redraw
    werase(stdscr);
    werase(win);
    box(win, 0, 0);
    getmaxyx(win, H, W);
    // Drawable region goes from y:1 to H-2 and x:1 to W-2

    // Spawn drone
    if(server_client_flag){
        // Client
        // Spawn drone
        wattron(win, COLOR_PAIR(4));
        mvwprintw(win,H/2,W/2 + 7,"+");
        wattroff(win, COLOR_PAIR(4));

        // Spawn drone from other application
        wattron(win, COLOR_PAIR(3));
        mvwprintw(win,H/2,W/2 - 7,"o");
        wattroff(win, COLOR_PAIR(3));
    }
    else {
        // Server
        // Spawn drone
        wattron(win, COLOR_PAIR(4));
        mvwprintw(win,H/2,W/2 - 7,"+");
        wattroff(win, COLOR_PAIR(4));

        // Spawn obstacle representing drone from other application
        wattron(win, COLOR_PAIR(3));
        mvwprintw(win,H/2,W/2 + 7,"o");
        wattroff(win, COLOR_PAIR(3));
    }

    refresh();
    wrefresh(win);

    draw_rect(win,6,6,H-7,W-7,1);
}

void change_obstacle_position_flag(){
    update_obstacles = 1;
}

void check_targets_reached(BlackboardMsg* positions, WINDOW* win, int* reached_targets, int fd_trs, int fd_npos_to_t, sem_t *log_sem){
    for(int i = 0; i < N_TARGETS; i++) {
        if(positions->drone_y == positions->targets[i][0] && 
            positions->drone_x == positions->targets[i][1]) {
                (*reached_targets)++; // Another target reached
                // Erase target position
                positions->targets[i][0] = -1;
                positions->targets[i][1] = -1;
                positions->reached_targets = *reached_targets;
                if(*reached_targets == N_TARGETS){
                    // All targets reached, spawn new targets
                    
                    *reached_targets = 0; // Reset reached targets counter
                    positions->reached_targets = *reached_targets;

                    // Asking targets program for new positions
                    int temp = positions->type;
                    positions->type = MSG_POS;

                    write(fd_npos_to_t, positions, sizeof(*positions));
                    read(fd_trs, positions, sizeof(*positions));

                    positions->type = temp;
                    
                    // Printing targets
                    for(int i = 0;i<N_TARGETS;i++){
                        wattron(win, COLOR_PAIR(2));
                        mvwprintw(win,positions->targets[i][0],positions->targets[i][1],"%d",i+1);
                        wattroff(win, COLOR_PAIR(2));
                        wrefresh(win);
                    }

                    write_log("application.log", "BLACKBOARD", "INFO", "All targets reached, new targets spawned", log_sem);

                }
        }
    }
}

// ------ used in drone.c ------

char* remove_white_space(char* string){
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

int load_parameters(const char* filename, double* drone_mass, double* air_resistance, double* integr_inter, int* rep_radius, double* max_force){
    FILE* file = fopen(filename, "r");
    if(!file){
        log_error("application.log", "DRONE", "file open", NULL);
        perror("FUNCTIONS.C line-139 file open");
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
                if(*drone_mass < 0){
                    printf("load_parameters: Invalid mass value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"AIR_RESISTANCE") == 0){
                *air_resistance = atoi(v);
                if(*air_resistance < 0){
                    printf("load_parameters: Invalid air resistance value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"INTEGRATION_INTERVAL") == 0){
                *integr_inter = atoi(v);
                if(*integr_inter < 0){
                    printf("load_parameters: Invalid integration interval\n");
                    return -1;
                }
            }
            else if(strcmp(k,"REPULSIVE_RADIUS") == 0){
                *rep_radius = atoi(v);
                if(*rep_radius < 0){
                    printf("load_parameters: Invalid repulsive radius\n");
                    return -1;
                }
            }
            else if(strcmp(k,"MAX_APPLIED_FORCE") == 0){
                *max_force = atoi(v);
                if(*max_force < 0){
                    printf("load_parameters: Invalid maximum force value\n");
                    return -1;
                }
            }
            else{
                printf("load_parameters: unknown parameter -> %s\n", k);
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
    double loop_next_drone_pos[2], int previous_drone_pos[2], int next_drone_pos[2], double* temp_x, double* temp_y, sem_t *log_sem){
    
    int distance = -1;
    // Computing distances among drone and obstacles too see if in range
    for(int i = 0;i<N_OBS;i++){
        if(obstacles[i][0] == -1 && obstacles[i][1] == -1)
            // Networked mode -> only one obstacle (others set to -1,-1)
            break;
        distance = sqrt(pow(drone_msg->new_drone_y - obstacles[i][0],2)+pow(drone_msg->new_drone_x - obstacles[i][1],2));
        if(distance <= ro){ // ro = 5
            // Drone is inside the obstacle repulsive area

            write_log("application.log", "DRONE", "INFO", "Drone within obstacle repulsive radius, computing repulsive forces", log_sem);

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

int move_drone(int fd_key, int fd_npos,DroneMsg* drone_msg, int next_drone_pos[2],double force_x, double force_y, double max_force, 
       double oblique_force_comp, double M, double K, double T, int borders[], int obstacles[N_OBS][2], int ro, sem_t *log_sem){

    fd_set rfds;
    struct timeval tv;
    int retval;
    int nfds = fd_key + 1;
    char received_key;
    int previous_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_prev_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_curr_drone_pos[2] = {drone_msg->new_drone_y, drone_msg->new_drone_x};
    double loop_next_drone_pos[2] = {0.0, 0.0};
    double epsilon = 1e-3;
    double temp_x = 0.0;
    double temp_y = 0.0;
    int delay = 40, control = 0, is_on_border = 0;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // Select checks every 100 ms
    sleep_ms(25); // Delay to ensure multiple consecutive clicks are taken into account correctly (the forces sum up)

    do{
        FD_ZERO(&rfds);
        FD_SET(fd_key, &rfds);
        // Reset timeout before each select call (select modifies tv)
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        retval = select(nfds, &rfds, NULL, NULL, &tv); // Returns only if a key has been pressed
        if(retval == -1){
            // EINTR means the call was interrupted by a signal (e.g., SIGALRM for heartbeat)
            // This is normal and we should just retry the select 
            // (since select is one of those functions that doesn't automatically restart even with SA_RESTART set)
            if(errno == EINTR){
                continue; // Retry select
            }
            // For other errors, log and exit
            log_error("application.log", "DRONE", "select", log_sem);
            perror("FUNCTIONS.C line-330 select");
            exit(EXIT_FAILURE);
        }
        else if(retval > 0 && FD_ISSET(fd_key, &rfds)){
            // Readable input
            ssize_t n = 0;
            n = read(fd_key,&received_key,1);
            if(n > 0){
                if(received_key == 'q'){
                    // Return to main loop to handle quit properly
                    drone_msg->type = MSG_QUIT;
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    drain_pipe(fd_key); // Clear any pending keys
                    return 1; // Signal quit to main loop
                }
                delay = 40;
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
                case 'o': 
                    // Artificial key sent when client obstacle position updated, but drone didn't move
                    // Just check for collision without moving the drone
                    force_x = force_x; 
                    force_y = force_y;
                    break;
                case 'd':
                    drone_msg->type = MSG_STOP;
                    write(fd_npos,drone_msg,sizeof(*drone_msg));
                    return 0;
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
            &temp_x, &temp_y, log_sem);

        // Checking if drone is within 5 pixels from the border
        while(drone_msg->new_drone_y <= 6 || drone_msg->new_drone_x <= 6 || 
            drone_msg->new_drone_y >= borders[0] || drone_msg->new_drone_x >= borders[1]){
            is_on_border = 1;
            control = 0;
            delay = 150;

            write_log("application.log", "DRONE", "WARNING", "Drone too close to border, applying repulsive forces", log_sem);

            float threshold = 6.0f;   // Distance from border where repulsion begins
            float k = max_force;      // Strength scale

            float fx = 0.0f;
            float fy = 0.0f;

            // Distance from borders:
            float d_left   = drone_msg->new_drone_x - 6;
            float d_right  = borders[1] - drone_msg->new_drone_x;
            float d_top    = drone_msg->new_drone_y - 6;
            float d_bottom = borders[0] - drone_msg->new_drone_y;

            // --- Horizontal repulsion ---
            if (d_left < threshold) {
                fx += k * (threshold - d_left) / threshold;  // force towards right
            }
            if (d_right < threshold) {
                fx -= k * (threshold - d_right) / threshold; // force towards left
            }

            // --- Vertical repulsion ---
            if (d_top < threshold) {
                fy += k * (threshold - d_top) / threshold;   // force towards down
            }
            if (d_bottom < threshold) {
                fy -= k * (threshold - d_bottom) / threshold;// // force towards up
            }

            // Output force
            force_x = fx;
            force_y = fy;
            
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
            else if(force_x < 0)
                force_x += 1;
            if(force_y > 0)
                force_y -= 1;
            else if(force_y < 0)
                force_y += 1;
        }

        // Depleting all forces applied
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
            return 0;
        }
        // Delay inserted to simulate a smoother slow down
        if(control == 4) delay = 70;
        sleep_ms(delay);

        // After first impulse forces are resetted
        force_x = 0.0;
        force_y = 0.0;

    }while(fabs(loop_next_drone_pos[1] - loop_prev_drone_pos[1]) > epsilon || 
            fabs(loop_next_drone_pos[0] - loop_prev_drone_pos[0]) > epsilon);
    
    // Signaling blackboard to interrupt waiting for new postions
    drone_msg->type = MSG_STOP;
    write(fd_npos,drone_msg,sizeof(*drone_msg));
    
    return 0;// Normal completion
}

// ------ used in obstcles.c & targets.c ------

int check_position(int new_y, int new_x, BlackboardMsg positions, int n_spawned_elem,
     int obstacles_spawned, int targets_spawned){
    // Check if new position is within 2 pixels from the drone
    if(new_y >= abs(positions.drone_y - 2) && new_y <= abs(positions.drone_y + 2) &&
       new_x >= abs(positions.drone_x - 2) && new_x <= abs(positions.drone_x + 2)){
        return 1; // Invalid position
    }

    if(obstacles_spawned){
        // Check if new position collides with other targets
        for(int i = 0;i<n_spawned_elem;i++){
            if(positions.targets[i][0] == new_y && positions.targets[i][1] == new_x){
                return 1; // Invalid position
            }
        }   
    }
    else{
        // Check if new position collides with other obstacles
        for(int i = 0;i<n_spawned_elem;i++){
            if(positions.obstacles[i][0] == new_y && positions.obstacles[i][1] == new_x){
                return 1; // Invalid position
            }
        }
    }

    if(targets_spawned){
        // Execute only if targets have already been spawned
        // Check if new position collides with targets
        for(int i = 0;i<N_TARGETS;i++){
            if(positions.targets[i][0] == new_y && positions.targets[i][1] == new_x){
                return 1; // Invalid position
            }
        }
    }

    if(obstacles_spawned){
        // If checking for targets spawning, ensure they don't spawn on obstacles
        for(int i = 0;i<N_OBS;i++){
            if(positions.obstacles[i][0] == new_y && positions.obstacles[i][1] == new_x){
                return 1; // Invalid position
            }
        }
    }

    return 0; // Valid position
}

ssize_t read_full(int fd, void* buf, size_t size) {
    size_t total = 0;
    while (total < size) {
        ssize_t n = read(fd, (char*)buf + total, size - total);
        if (n <= 0) return n; // error or pipe closed
        total += n;
    }
    return total;
}

// ------ used in master.c ------

int spawn(const char *prog, char *const argv[]) {
    pid_t pid = fork();
    if (pid < 0) { 
        log_error("application.log", "MASTER", "fork", NULL);
        perror("FUNCTIONS.C line-631 fork"); exit(EXIT_FAILURE); 
    }
    if (pid == 0) {
        execvp(prog, argv);
        log_error("application.log", "MASTER", "execvp", NULL);
        perror("FUNCTIONS.C line-636 execvp"); 
        exit(EXIT_FAILURE);
    }
    return pid;
}

// ------ used in master.c & input_manager.c & blackboard.c & obstacles.c & targets.c ------

void write_process_pid(const char* log_filename, const char* process_name,
     pid_t pid, sem_t *log_sem){
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "[%Y-%m-%d %H:%M:%S]", t);
    char log_message[512];
    snprintf(log_message, sizeof(log_message), "%s [%s] [%d]", time_str, process_name, (int)pid);

    // Ensure exclusive access to the log file
    sem_wait(log_sem);
    FILE* log_file = fopen(log_filename, "a"); // Append mode -> seek end of file
    if(!log_file){
        perror("FUNCTIONS.C line-657 fopen");
        exit(EXIT_FAILURE);
    }
    fprintf(log_file, "%s\n", log_message);
    fflush(log_file); // Ensure data is written to file
    fclose(log_file);
    sem_post(log_sem);
}

void write_log(const char* log_filename, const char* process_name,
     const char* level, const char* message, sem_t *log_sem){
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "[%Y-%m-%d %H:%M:%S]", t);
    char log_message[512];
    snprintf(log_message, sizeof(log_message), "%s [%s] [%s] %s", time_str, process_name, level, message);

    // Ensure exclusive access to the log file
    sem_wait(log_sem);
    FILE* log_file = fopen(log_filename, "a"); // Append mode -> seek end of file
    if(!log_file){
        perror("FUNCTIONS.C line-679 fopen");
        exit(EXIT_FAILURE);
    }
    fprintf(log_file, "%s\n", log_message);
    fflush(log_file); // Ensure data is written to file
    fclose(log_file);
    sem_post(log_sem);
}

void log_error(const char* log_filename, const char* process_name,
     const char* context, sem_t *log_sem){
    char error_msg[512];
    snprintf(error_msg, sizeof(error_msg), "%s: %s", context, strerror(errno));
    write_log(log_filename, process_name, "ERROR", error_msg, log_sem);
}

// ------ heartbeat helpers ------

void heartbeat_signal_handler(int signo) {
    (void)signo;
    heartbeat_due = 1;
}

void setup_heartbeat_itimer(int interval_sec) {
    // Use sigaction with SA_RESTART to avoid interrupting blocking syscalls
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = heartbeat_signal_handler;
    sa.sa_flags = SA_RESTART;  // Automatically restart interrupted syscalls
    sigaction(SIGALRM, &sa, NULL);
    
    struct itimerval timer;
    timer.it_interval.tv_sec = interval_sec;
    timer.it_interval.tv_usec = 0;
    timer.it_value.tv_sec = interval_sec;
    timer.it_value.tv_usec = 0;
    if (setitimer(ITIMER_REAL, &timer, NULL) == -1) {
        perror("FUNCTIONS.C line-716 setitimer");
        // Do not exit; heartbeat is optional. Log if available.
    }
}

int setup_heartbeat_posix_timer(int interval_sec, int signo) {
    // Use sigaction with SA_RESTART to avoid interrupting blocking syscalls
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = heartbeat_signal_handler;
    sa.sa_flags = SA_RESTART;  // Automatically restart interrupted syscalls
    sigaction(signo, &sa, NULL);
    
    struct sigevent sev;
    memset(&sev, 0, sizeof(sev));
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = signo;
    sev.sigev_value.sival_ptr = &heartbeat_timer_id;
    if (timer_create(CLOCK_REALTIME, &sev, &heartbeat_timer_id) == -1) {
        perror("FUNCTIONS.C line-735 timer_create");
        return -1;
    }

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_interval.tv_sec = interval_sec;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = interval_sec;
    its.it_value.tv_nsec = 0;
    if (timer_settime(heartbeat_timer_id, 0, &its, NULL) == -1) {
        perror("FUNCTIONS.C line-746 timer_settime");
        return -1;
    }
    return 0;
}

void send_heartbeat_if_due(int fd_watchdog, const char* process_name, sem_t *log_sem) {
    if (heartbeat_due) {
        heartbeat_due = 0;
        pid_t pid = getpid();
        ssize_t n = write(fd_watchdog, &pid, sizeof(pid));
        if (n != (ssize_t)sizeof(pid)) {
            // Non-fatal: watchdog may be down; log for visibility
            if (log_sem) {
                log_error("watchdog.log", process_name, "heartbeat write", log_sem);
            }
        }
    }
}

// ------ used in socket_manager.c ------

void analyze_position_n_size_and_prepare_message(BlackboardMsg positions,char* buffer_output, int wind_H){
    if(positions.type == MSG_WSIZE) {
        // Window size sent as width, height
        const int width = positions.border_x + 7;  // Stored value is border width (cols) without margins
        const int height = positions.border_y + 7; // Stored value is border height (rows) without margins
        snprintf(buffer_output, 256, "size %d, %d\n", width, height);
    }
    else if(positions.type == MSG_NPOS) {
        // Drone position sent as x, y (after converting y to bottom-left frame expected on the wire)
        const int x = positions.drone_x;
        const int y = wind_H - positions.drone_y; // Convert from top-left origin to bottom-left
        snprintf(buffer_output, 256, "%d, %d\n", x, y);
    } 
    else {
        log_error("application.log", "SOCKET_MANAGER", "Unknown message type received by blackboard", NULL);
    }
}

void error(int newsockfd, int sockfd, const char *msg, sem_t *log_sem) {
    write_log("application.log", "SOCKET_MANAGER", "ERROR", msg, log_sem);
    if(newsockfd != -1)
        close(newsockfd);
    close(sockfd);
    exit(EXIT_FAILURE);
}

ssize_t read_line(int fd, char *buf, size_t maxlen) {
    size_t i = 0;
    char c;

    while (i < maxlen - 1) {
        ssize_t n = read(fd, &c, 1);
        if (n == 1) {
            buf[i++] = c;
            if (c == '\n') break;
        } else if (n == 0) {
            return 0;   // connection closed
        } else {
            return -1;  // error
        }
    }
    buf[i] = '\0';
    return i;
}