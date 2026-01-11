# ----------------------------
#   COMPILATION SETTINGS
# ----------------------------

CC      = gcc
CFLAGS  = -Wall -Wextra -D_POSIX_C_SOURCE=200809L
LIBS    = -lncurses -lm -lrt

# Common source file
COMMON  = functions.c

# Executables to build
TARGETS = master blackboard drone input_manager obstacles targets watchdog blackboardserver blackboardclient socket_manager

# Default target
all: $(TARGETS)
	@echo "Running master..."
	./master

# ----------------------------
#   BUILD RULES
# ----------------------------

master: master.c $(COMMON)
	$(CC) $(CFLAGS) master.c $(COMMON) -o master $(LIBS)

blackboard: blackboard.c $(COMMON)
	$(CC) $(CFLAGS) blackboard.c $(COMMON) -o blackboard $(LIBS)

drone: drone.c $(COMMON)
	$(CC) $(CFLAGS) drone.c $(COMMON) -o drone $(LIBS)

input_manager: input_manager.c $(COMMON)
	$(CC) $(CFLAGS) input_manager.c $(COMMON) -o input_manager $(LIBS)

obstacles: obstacles.c $(COMMON)
	$(CC) $(CFLAGS) obstacles.c $(COMMON) -o obstacles $(LIBS)

targets: targets.c $(COMMON)
	$(CC) $(CFLAGS) targets.c $(COMMON) -o targets $(LIBS)

watchdog: watchdog.c $(COMMON)
	$(CC) $(CFLAGS) watchdog.c $(COMMON) -o watchdog $(LIBS)

blackboardserver: blackboardserver.c $(COMMON)
	$(CC) $(CFLAGS) blackboardserver.c $(COMMON) -o blackboardserver $(LIBS)

blackboardclient: blackboardclient.c $(COMMON)
	$(CC) $(CFLAGS) blackboardclient.c $(COMMON) -o blackboardclient $(LIBS)

socket_manager: socket_manager.c $(COMMON)
	$(CC) $(CFLAGS) socket_manager.c $(COMMON) -o socket_manager $(LIBS)

# ----------------------------
#   UTILITY TARGETS
# ----------------------------

clean:
	rm -f $(TARGETS) *.o

.PHONY: all clean
