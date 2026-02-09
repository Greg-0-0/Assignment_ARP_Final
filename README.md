# 3rd ASSIGNMENT of ADVANCED ROBOT PROGRAMMING

This project consists of a **multi-process application** that implements an **interactive drone operation simulator**. The application can be executed either in **standa-alone mode** or in **networked mode**. In the second modality, the application establishes a client-server connection with another software, allowing each application to pass drone controls to the other end. The communication is implemented using a socket channel: before the startup, the application asks for the necessary information to establish the connection, such as the server IP address in case of the client side, and the communication port used by both server and client.

---
## Stand-alone mode

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

## Networked mode

The application works differently wether it is executed as client or server.
The communication between the two sides follows a precise deterministic protocol, implementing an acknowledgment mechanic for each message sent.
First, the server creates the socket channel, and waits for the connection of the client. Once the connection has been established, the server provides the dimensions of the field (**character window**), which the client side has to recreate. Afterwards, the two ends keep exchanging the position of their respective drones, updating their local representation of the game field. The client drone functions as an obstacle, repelling the server drone whenever it enters its repulsive range. Indeed, the goal of the server drone is to escape the moving obstacle, avoiding to be cornered.

At startup, the software creates:
- a **character window** using the **ncurses** library,
- a **smaller input window** used to control the drone movement.
In this modality the watchdog process, as the obstacles and targets, are disabled to focus on the communication between the two applications.

The main window displays a boxed area inside which the drone can move. Inside the field there are:
- the **drone**, initially positioned slightly on the left relaitve to the center (blue `+`),
- the drone of the other application represented as an **obstacle** (orange `o`).

The goal depends on which part is being considered:
- the server has to escape the drone of the obstacle, that applies a **repulsive force** on it,
- the client has to corner the server drone using its **repulsive force**.
The window borders also exert a repulsive force, simulating **geo-fences** used in real drone operations.

The second window is used to input commands for the drone movement. Eight directional keys apply forces to the drone, moving it in the corresponding direction. Two additional commands allow the user to **stop the drone** or **quit the application**.

The input window also displays the **current drone position**, expressed in characters.

---

## Processes

- **master**  
  Spawns all the processes and provides unnamed pipes for IPC.

- **blackboard**
  Process used only in **stand-alone mode**.
  Manages the character window implemented with ncurses, including window resizing and entity spawning.  
  It communicates with the obstacles and targets to obtain their coordinates and exchanges information with the drone to update its position.

- **drone**  
  Implements the dynamic behavior of the application, managing drone movement and repulsive forces from obstacles and borders.  
  It applies forces based on the keys received from the `input_manager` and sends its updated position to the blackboard.

- **input_manager**  
  Spawns the input window where user commands are received and forwarded to the drone process.  
  It also updates the drone position and the score displayed in the window.

- **obstacles**  
  Process used only in **stand-alone mode**.
  Computes obstacle coordinates every 10 seconds and communicates them to the blackboard.

- **targets**  
  Process used only in **stand-alone mode**.
  Computes target coordinates at startup and every time all targets have been reached, then sends them to the blackboard.

- **watchdog**  
  Process used only in **stand-alone mode**.
  Checks every 3 seconds whether the other processes have sent a heartbeat.  
  Processes are marked as *recovered* if notified correctly, otherwise they are considered *timed out*.

- **socket_manager**
  Process used only in **networked mode**.
  Manages the usage of the socket channel both in client and server mode, ensuring the communication protocol is respected.
  It takes care of the information received from the other side, as well as sending the necessary data to the other end of the communication channel.

- **blackboardserver**
  Process used only in **networked mode**.
  Manages the character window implemented with ncurses, sending its sizes to the client side. The resizing feature is disabled in this playmode. It communicates with the socket_manager to exchange information with the client side, such as sending the position of its drone and receiving the position of the client drone (seen as an obstacle).

- **blackboardclient**
  Process used only in **networked mode**.
  Manages the character window implemented with ncurses, receiving its sizes from the server side. The resizing feature is disabled in this playmode. It communicates with the socket_manager to exchange information with the server side, such as sending the position of its drone and receiving the position of the server drone.

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
  Builds all programs, links the required libraries, cleans the log files, and executes the application.

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
  Builds all executables, erases log files, and launches the application.

- make clean  
  Removes all compiled executables and log files.

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

### Function used in socket_manager

- analyze_position_n_size_and_prepare_message
  Builds the messages relative to the dimensions of the ncurses window or the drone positions.

- error
  Separate function used by the the socket_manager to print custom error messages in the corresponding log file.

- read_line
  Helper function to read message received on the socket character by character

---

### Tested with


---

## Author

Daneri Gregorio

# -----------> Have fun!!! <-----------
