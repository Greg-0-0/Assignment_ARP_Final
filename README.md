                                                                                                 
#  1st ASSIGNMENT of ADVANCED ROBOT PROGRAMMING 



This project implements a multi-process application where a master process runs multiple subprocesses to carry out various tasks.
The main goal is to move a drone (+) inside a boxed area filled with ten repuslive obstacles (o) and reach all the other ten targets (T).
The border, displayed with a red rectangle, has a repulsive effect as well.

# Components
- master.c ->           spawns all the subprocesses
- blackboard.c ->       manages the GUI implemented with ncurses library
- drone.c ->            implements the dynamic of drone, obstacles and borders
- input_manager.c ->    manages user inputs to control the drone movements
- obstacles.c ->        generates static repulsive obstacles
- targets.c ->          generates static targets
- functions.c/h ->       implements the functions used by the processes
- parameters.txt ->     file with constant parameters used by drone process
- code_architecture.pdf a graphic representation of the project structure and high level dependecies among processes

# Libraries
- ncurses
- math

# Requirements
- install ncurses library -> sudo apt install libncurses-dev

# Instructions to run
- make ->       executes Makefile
- make clean -> removes all executables and executes Makefile

# Instructions to use
Keys:
    - w -> moves drone towards top-left
    - r -> moves drone towards top-right
    - x -> moves drone towards bottom-left
    - v -> moves drone towards bottom-right
    - c -> moves drone towards down
    - e -> moves drone towards up
    - s -> moves drone towards left
    - f -> moves drone towards right
    - d -> stops the drone
    - q -> quit
SUGGESTION: use half of the screen (or two thirds) to show the blackboard screen and the remaniing half (or one third) to display the input manager

# Functioning
The initial master process spawns the blackboard process, input_manager process, drone process, obstacles process and targets process. The processes use unnamed pipes to communicate. The blackboard, as first step, sends its border values to both obstacles and targets processes, which randomly define their positions and send them back to the blackboard. With the newly obtained data, the blackboard spawns the entities, along with the drone. Afterwards it sends these coordinates to the drone and it keeps waiting until the drone position is update by the user input. In this case the pressed key is passed to the drone from the input_manager using pipes, and then the drone computes its dynamic, eventually using the data received from the blackboard to calculate possible repulsive forces of the obstacles and/or borders. The resulting coordinates are sent to the blackboard which updates the graphical position of the drone.

To comprehend in more depth how the application works I suggest viewing the code comments.

# Functions
Functions are implemented in file "functions.c" and distributed across the programs using the header library "function.h".
Functions used in blackboard.c:
- draw_rect: draws the red rectanlge in the ncurses window to identify the inner borders (crossable with drone, but with repulsion effect)
- layout_and_draw: creates the ncurses window with the outer borders (not crossable)
Functions used in drone.c:
- remove_white_space: removes white spaces from file parameters.txt when reading it line by line
- load_parameter: parses parameters.txt to load parameters value
- sleep_ms: allows the process to sleep for a defined amount of milliseconds (used in drone dynamic to achieve smoother motions)
- drain_pipe: gurantees that pipes are emptied before reusing them, to avoid undesired data reading
- compute_repulsive_forces: implements the dynamic of repulsion effect of obstacles
- move_drone: implements the dynamic of drone movement and repulsion effect of borders
Function used in obstacles.c and targets.c:
- read_full: ensures that the entire set of bytes passed through a pipe is read (used to avoid undesired behaviour from pipe reading)
Function used in master.c:
- spawn: forks new processes and use them to run new executables with execvp

# Author
Daneri Gregorio

# -----------> Have fun!!! <-----------