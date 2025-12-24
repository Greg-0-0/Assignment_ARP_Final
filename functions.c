#include"functions.h"

// Define the global variable (declared as extern in .h)
volatile sig_atomic_t update_obstacles = 0;

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

void change_obstacle_position_flag(){
    update_obstacles = 1;
}

void check_targets_reached(BlackboardMsg* positions, WINDOW* win, int* reached_targets, int fd_trs, int fd_npos_to_t){
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
                if(*drone_mass < 0){
                    printf("Invalid mass value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"AIR_RESISTANCE") == 0){
                *air_resistance = atoi(v);
                if(*air_resistance < 0){
                    printf("Invalid air resistance value\n");
                    return -1;
                }
            }
            else if(strcmp(k,"INTEGRATION_INTERVAL") == 0){
                *integr_inter = atoi(v);
                if(*integr_inter < 0){
                    printf("Invalid integration interval\n");
                    return -1;
                }
            }
            else if(strcmp(k,"REPULSIVE_RADIUS") == 0){
                *rep_radius = atoi(v);
                if(*rep_radius < 0){
                    printf("Invalid repulsive radius\n");
                    return -1;
                }
            }
            else if(strcmp(k,"MAX_APPLIED_FORCE") == 0){
                *max_force = atoi(v);
                if(*max_force < 0){
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
       double oblique_force_comp, double M, double K, double T, int borders[], int obstacles[N_OBS][2], int ro){

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
    sleep_ms(250); // Delay to ensure multiple consecutive clicks are taken into account correctly (the forces sum up)

    do{
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
        while(drone_msg->new_drone_y <= 6 || drone_msg->new_drone_x <= 6 || 
            drone_msg->new_drone_y >= borders[0] || drone_msg->new_drone_x >= borders[1]){
            is_on_border = 1;
            control = 0;
            delay = 150;

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
            return;
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
    
    return;
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

void spawn(const char *prog, char *const argv[]) {
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(EXIT_FAILURE); }
    if (pid == 0) {
        execvp(prog, argv);
        perror("execvp"); 
        exit(EXIT_FAILURE);
    }
}

