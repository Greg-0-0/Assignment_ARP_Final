                                                                                                 
# # 1st ASSIGNMENT for ADVANCED ROBOT PROGRAMMING 



This project implements a multi-process application where a master process runs multiple subprocesses to carry out various tasks.
The main goal is to move a drone inside a boxed area filled with repuslive obstacles (o) and reach all the targets (T).
The border has a repulsive effect as well.

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
- keys:
    w -> moves drone towards top-left
    r -> moves drone towards top-right
    x -> moves drone towards bottom-left
    v -> moves drone towards bottom-right
    c -> moves drone towards down
    e -> moves drone towards up
    s -> moves drone towards left
    f -> moves drone towards right
    d -> stops the drone
    q -> quit

# Possible bugs
- ncurses windows spawns obstacles and targets  -> resize ncurses window to not full-screen,
  only on a certain part of the screen             close program and then run it again

# Author
Daneri Gregorio

# -----------> Have fun!!! <-----------