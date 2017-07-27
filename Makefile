CC=g++
STD=-std=c++11
CFLAGS=-g -D__USE_W32_SOCKETS -D_WIN32_WINNT=0x0501 -L/usr/local/lib -lkrpc -lprotobuf -lmswsock -lws2_32 -lm -Wall
DEPS = 
		
all:
	

auto: auto.cpp
	$(CC) $(STD) auto.cpp -o auto.exe $(CFLAGS)
	
data: data.cpp
	$(CC) $(STD) data.cpp -o data.exe $(CFLAGS)