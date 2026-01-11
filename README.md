# 2nd ASSIGNMENT of ADVANCED ROBOT PROGRAMMING

This project consists of a **multi-process application** that implements an **interactive drone operation simulator**.

At startup, the software spawns:
- a **character window** using the **ncurses** library,
- a **smaller input window** used to control the drone movement, and
- a **small output window** used to display the interactions between the processes and the watchdog.

The main window displays a boxed area inside which the drone can move. Inside the field there are:
- **10 obstacles** (orange `o`), spawned randomly,
- **9 targets** (green numbers), also placed randomly,
- the **drone**, initially positioned at the center (blue `+`).

The main goal is to make the drone reach all targets **in sequence**, while avoiding the obstacles.  
Obstacles apply a **repulsive force** to the drone when it gets too close. The window borders also exert a repulsive force, simulating **geo-fences** used in real drone operations. The drone can move freely only inside the **red rectangle**.

Obstacles despawn and respawn randomly approximately every **10 seconds**, while targets are regenerated only after **all of them have been reached**.

The second window is used to input commands for the drone movement. Eight directional keys apply forces to the drone, moving it in the corresponding direction. Two additional commands allow the user to **stop the drone** or **quit the application**.

This window also displays:
- the **current score**, represented by the number of targets reached (which resets after all targets are collected),
- the **current drone position**, expressed in characters.

The last window displays various messages communicating whether every process has sent its heartbeat to the watchdog in time, it missed the timer deadline of **3 seconds**, or it recovered, after timing out.

---

## Processes

- **master**  
  Spawns all the processes and provides unnamed pipes for IPC.

- **blackboard**  
  Manages the character window implemented with ncurses, including window resizing and entity spawning.  
  It communicates with the obstacles and targets to obtain their coordinates and exchanges information with the drone to update its position.

- **drone**  
  Implements the dynamic behavior of the application, managing drone movement and repulsive forces from obstacles and borders.  
  It applies forces based on the keys received from the `input_manager` and sends its updated position to the blackboard.

- **input_manager**  
  Spawns the input window where user commands are received and forwarded to the drone process.  
  It also updates the drone position and the score displayed in the window.

- **obstacles**  
  Computes obstacle coordinates every 10 seconds and communicates them to the blackboard.

- **targets**  
  Computes target coordinates at startup and every time all targets have been reached, then sends them to the blackboard.

- **watchdog**  
  Checks every 3 seconds whether the other processes have sent a heartbeat.  
  Processes are marked as *recovered* if notified correctly, otherwise they are considered *timed out*.

---

## Additional Components

- **functions.c / functions.h**  
  Files used to define, implement, and organize functions shared among processes.

- **parameters.txt**  
  Defines constant parameters used by the drone process.

- **code_architecture_assignment2.pdf**  
  High-level graphical representation of the project structure, including process connections via pipes.

- **application.log**  
  Log file generated at runtime containing information about process start/termination, pressed keys, targets reached, obstacle respawns, repulsion effects, warnings, and errors.

- **watchdog.log**  
  Log file used to inspect process communication with the watchdog (timeouts and recoveries).

- **processes.log**  
  Log file displaying the process identifiers (PIDs) of each program at application startup.

- **Makefile**  
  Builds all programs, links the required libraries, and executes the application.

---

## Libraries

- **ncurses** (`-lncurses`)
- **math**
- **real-time extension** (`-lrt`)

---

## Requirements

- Install ncurses library:
  ```bash
  sudo apt install libncurses-dev

---

## Instructions to Run

- make  
  Builds all executables and launches the application.

- make clean  
  Removes all compiled executables.

---

## Instructions to Use

For a better experience, use half (or two thirds) of your screen for the character window and the remaining space for the input window.

### Controls

- w -> moves drone towards top-left  
- r -> moves drone towards top-right  
- x -> moves drone towards bottom-left  
- v -> moves drone towards bottom-right  
- c -> moves drone down  
- e -> moves drone up  
- s -> moves drone left  
- f -> moves drone right  
- d -> stops the drone  
- q -> quit the application  

---

## Functions

Functions are implemented in the file "functions.c" and shared across programs using the header file "functions.h".

### Functions used in blackboard

- draw_rect  
  Draws the red rectangle identifying the inner borders (crossable by the drone, but with repulsion).

- layout_and_draw  
  Creates the ncurses window with the outer borders (not crossable).

- change_obstacle_position_flag  
  Sets a flag to change obstacle positions. This function is defined as a signal handler and is triggered every 10 seconds.

- check_target_reached  
  Checks whether all targets have been reached by the drone and, if so, requests new targets from the targets process.

### Functions used in drone

- remove_white_space  
  Removes white spaces when reading the parameters.txt file line by line.

- load_parameter  
  Parses parameters.txt to load parameter values.

- sleep_ms  
  Suspends execution for a specified number of milliseconds, used to achieve smoother motion.

- drain_pipe  
  Ensures pipes are emptied before reuse to avoid reading stale data.

- compute_repulsive_forces  
  Implements the repulsion dynamics generated by obstacles.

- move_drone  
  Implements drone movement and border repulsion dynamics.

### Functions used in obstacles and targets

- check_position  
  Checks that newly spawned obstacles or targets do not overlap with the drone or other entities.

- read_full  
  Ensures that all bytes sent through a pipe are correctly read.

### Functions used in master

- spawn  
  Forks and executes new processes using execvp, providing the required arguments.

### Functions used by multiple processes

- write_process_pid  
  Writes the PID of the calling process to the processes.log file at startup.

- write_log  
  Writes messages to application.log and watchdog.log, including events, warnings, and errors.

- log_error  
  Helper function used to log errors through write_log.

### Heartbeat helpers

- heartbeat_signal_handler  
  Sets a flag indicating that a heartbeat must be sent.

- setup_heartbeat_itimer  
  Sets a timer to trigger heartbeat signals.

- setup_heartbeat_posix_timer  
  Used by the blackboard process to avoid conflicts with SIGALRM by using a POSIX real-time signal.

- send_heartbeat_if_due  
  Sends a heartbeat to the watchdog if the corresponding flag is set.

---

## Corrections

- Added colors for obstacles, targets, and the drone.
- Reviewed application termination to remove potential bugs.

---

## Author

Daneri Gregorio

# -----------> Have fun!!! <-----------