######################################################################
#
# Author: Shaohui Sun
#
######################################################################


CXX = g++

INCLUDE_PATH = -I/usr/local/include -I/usr/local/include/opencv 
LDLIB_PATH = -L/usr/local/lib

CXXFLAGS = -O2 -Wall ${INCLUDE_PATH}

LDFLAGS = ${LDLIB_PATH} -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_features2d

OBJS = parabolicMask.o line.o laneExtraction.o laneModeling.o ransacLine2D.o

all: track

track: track.o ${OBJS}
	${CXX} ${LDFLAGS} -o $@ $^ ${LDFLAGS}

%.o: ./%.cpp
	${CXX} -c ${CXXFLAGS} $<
	
clean:
	rm -rf *.o track
