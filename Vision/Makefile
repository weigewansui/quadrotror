all:../bin/Vision_state

../bin/Vision_state:
	g++ detectionAlgo.cpp -o ../bin/Vision_state `pkg-config --cflags --libs opencv` `pkg-config --cflags --libs lcm` ../src/util.o ../src/filter_util.o
	g++ video_capture.cpp -o ../bin/capture `pkg-config --cflags --libs opencv` -lboost_system -lboost_thread