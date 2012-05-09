###############################################################
#
# Purpose: Makefile for "jane"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################
#!/bin/sh

#Capital-letter phrases: MACROS <MACRONAME> = <macro_string>, similar to #define
TARGET = jane


INCLUDE_DIRS = -I../../include -I../../../Framework/include 

#name of compiler 
CXX = g++
.SUFFIXES: .o .h .c .hpp .cpp


# Linking (add flags as needed)
#LFLAGS = `pkg-config --libs opencv` -lpthread -ljpeg -lrt
LDFLAGS =  -lpthread -ljpeg -lrt  # LFLAGS or LDFLAGS: flags used in linking. NOTE:` `meaning:contains command not ' ' meaning: contains string.
#CXXFLAGS += `pkg-config --cflags opencv`                    # CFLAGS or CXXFLAGS: used in compiling and creating object files. includes both "-Wall" and "-c":creates .o files
CXXFLAGS += -g -DLINUX -Wall -c $(INCLUDE_DIRS)             # -Wall:print all warnings. 


OBJECTS = Jane_DarwinDrive_main.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo   #remember the TAB in the beginning of each command!!!

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

# dependencies for target:darwin.a  -o is the option to create executable rather than a.out
$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(LDFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET) 
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/Jane_DarwinDrive_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
