
TARGET_EXEC = GyroBno.exe
BUILD_DIR = ./bin
SRC_DIR = ./src
CC = gcc

VPATH = src

# C Flags
CFLAGS			+= -Wall 
CFLAGS			+= -g

LDFLAGS = -l bcm2835  -lm

# define the C source files
SRCS				+= GyroBno.c
SRCS				+= apiBno.c
SRCS				+= shmcmn.c

OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o)
#DEPS := $(OBJS:.o=.d)

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

#-include $(DEPS)

MKDIR_P ?= mkdir -p
