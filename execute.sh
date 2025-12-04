gcc master.c -o master
gcc -Wall -Wextra blackboard.c -lncurses -o blackboard
gcc drone.c -o drone -lm
gcc input_manager.c -lncurses -o input_manager
gcc obstacles.c -o obstacles
gcc targets.c -o targets
./master