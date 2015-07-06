# Makefile template for shared library
 
CC = gcc # C compiler
CFLAGS = -Wall -Wextra -O3 -g  # C flags

LDFLAGS = -lm -lrt -lpthread -lpigpio

INC = -I./include

RM = rm -f  # rm command
TARGET = slam
 
SRCS = src/bmp.c src/matrices.c src/ekf.c src/to_frame.c src/from_frame.c src/scan.c src/inv_scan.c src/move.c src/observe.c src/inv_observe.c src/hough.c src/decode_packet.c src/read_serial.c src/read_sensors.c src/quaternion.c src/vector.c src/joy.c src/motors.c
#include "../includes/state.h"

 # source files
OBJS = $(SRCS:.c=.o)
 
.PHONY: all
all: ${TARGET}
 
$(TARGET): $(OBJS) main.c
	$(CC) main.c $(OBJS) $(CFLAGS) $(INC) ${LDFLAGS} -o $(TARGET)

$(SRCS:.c=.d):%.d:%.c
	$(CC) $(CFLAGS) $(INC) -MM $< >$@
 
include $(SRCS:.c=.d)
 
.PHONY: clean
clean:
	-${RM} ${TARGET_LIB} ${OBJS} 
