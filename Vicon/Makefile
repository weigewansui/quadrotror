CUR_DIR = $(shell pwd)
CFLAGS = `pkg-config --cflags lcm`
LDFLAGS = `pkg-config --libs lcm` ../src/filter_util.o ../src/util.o

vicon_lib = $(CUR_DIR:%/HLP/Vicon=%/Vicon/libs)

../bin/Vicon_state:
	g++ $(CFLAGS) Vicon_state_data.cpp -o ../bin/Vicon_state -Wl,-rpath=../../Vicon/libs -L$(vicon_lib) -lViconDataStreamSDK_CPP -lDebugServices $(LDFLAGS)