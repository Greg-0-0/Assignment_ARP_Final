#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<fcntl.h>

// Function to run separte processes
static void spawn(const char *prog, char *const argv[]) {
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(EXIT_FAILURE); }
    if (pid == 0) {
        execvp(prog, argv);
        perror("execvp"); 
        exit(EXIT_FAILURE);
    }
}

int main(){
    // Arrays to store file descriptors
    int pipe_I_to_D[2];
    int pipe_D_to_B_Req[2];
    int pipe_B_to_D_Pos[2];
    int pipe_D_to_B_NPos[2];
    int pipe_O_to_B[2];
    int pipe_B_to_O[2];

    if(pipe(pipe_I_to_D) < 0 || pipe(pipe_D_to_B_Req) < 0 || pipe(pipe_B_to_D_Pos) < 0 ||
        pipe(pipe_D_to_B_NPos) < 0 || pipe(pipe_O_to_B) < 0 || pipe(pipe_B_to_O)){
        perror("pipe");
        exit(EXIT_FAILURE);
    }

    // ---------- spawn blackboard ----------
    char argb1[32], argb2[32], argb3[32], argb4[32], argb5[32];
    snprintf(argb1, sizeof argb1, "%d", pipe_D_to_B_Req[0]); // read requests from drone
    snprintf(argb2, sizeof argb2, "%d", pipe_D_to_B_NPos[0]); // read new positions from drone
    snprintf(argb3, sizeof argb3, "%d", pipe_B_to_D_Pos[1]); // replies to drone
    snprintf(argb4, sizeof argb4, "%d", pipe_B_to_O[1]); // writes positions to obstacles
    snprintf(argb5, sizeof argb5, "%d", pipe_O_to_B[0]); // reads positions from obstacles
    char *args_blackboard[] = { "konsole", "-e", "./blackboard", argb1, argb2, argb3, argb4, argb5, NULL };
    spawn(args_blackboard[0], args_blackboard);

    // ---------- spawn drone ----------
    char argd1[32], argd2[32], argd3[32], argd4[32] ;
    snprintf(argd1, sizeof argd1, "%d", pipe_D_to_B_Req[1]); // writes requests to blackboard
    snprintf(argd2, sizeof argd2, "%d", pipe_D_to_B_NPos[1]); // writes new positions to blackboard
    snprintf(argd3, sizeof argd3, "%d", pipe_B_to_D_Pos[0]); // reads replies from blackboard
    snprintf(argd4, sizeof argd4, "%d", pipe_I_to_D[0]); // reads commands from input_manager
    char *args_drone[] = { "konsole", "-e", "./drone", argd1, argd2, argd3, argd4, NULL };
    spawn(args_drone[0], args_drone);

    // ---------- spawn input_manager ----------
    char argi1[32];
    snprintf(argi1, sizeof argi1, "%d", pipe_I_to_D[1]); // writes commands to drone
    char *args_input_manager[] = { "konsole", "-e", "./input_manager", argi1, NULL };
    spawn(args_input_manager[0], args_input_manager);

    // ---------- spawn obstacles ----------
    char argo1[32], argo2[32];
    snprintf(argo1, sizeof argo1, "%d", pipe_B_to_O[0]); // reads positions from blackboard
    snprintf(argo2, sizeof argo2, "%d", pipe_O_to_B[1]); // writes positions to blackboard
    char *args_obstacles[] = { "konsole", "-e", "./obstacles", argo1, argo2, NULL };
    spawn(args_obstacles[0], args_obstacles);

    // Closing all pipes
    close(pipe_I_to_D[0]); close(pipe_I_to_D[1]);
    close(pipe_D_to_B_Req[0]); close(pipe_D_to_B_Req[1]);
    close(pipe_B_to_D_Pos[0]); close(pipe_B_to_D_Pos[1]);
    close(pipe_D_to_B_NPos[0]); close(pipe_D_to_B_NPos[1]);
    close(pipe_B_to_O[0]); close(pipe_B_to_O[1]);
    close(pipe_O_to_B[0]); close(pipe_O_to_B[1]);
    return 0;
}