###############################################################
#
# Purpose: Makefile for "jane"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################
#!/bin/sh


TARGET = jane

INCLUDE_DIRS = -I../../include -I../../../Framework/include -I../../opencv 

CXX = g++
#CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
CXXFLAGS += -g -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)

# Linking (add flags as needed)
LFLAGS = -lpthread -ljpeg -lrt 'pkg-config --libs opencv'



OBJECTS = Jane_DarwinDrive_main.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(LFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/Jane_DarwinDrive_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
