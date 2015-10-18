main.o:main.c
	gcc -c main.c

asctecCommIntf.o:asctecCommIntf.c
	gcc -c asctecCommIntf.c

test: %.o
	gcc -o test asctecCommIntf.o main.o -pthread