CC = clang++
CFLAGS = -std=c++11 -Wall -O2 -fno-exceptions -fno-rtti 
#CFLAGS = -Wall -fno-exceptions -fno-rtti 

D_CFLAGS = -DDRAWSTUFF

#for CYGWIN
#LDLIBS = -lm -L spd/lib -lspd -L/usr/local/lib -lode -ldrawstuff -lgsl -lgslcblas\
#-lstdc++ -lComctl32 -lkernel32 -luser32 -lgdi32 -lOpenGL32 -lGlu32\
#/usr/local/lib/resources.RES
#INLIBS = 

#for QNX
#LDLIBS = -lm -L spd/lib -lspd -L/usr/local/lib -lode -ldrawstuff -lgsl -lgslcblas \
#-L/opt/X11R6/lib -lGL -lGLU -lsocket
#INLIBS = -I/usr/local/include -I/opt/X11R6/include

#for LINUX(galileo)
LDLIBS = -lm -L spd/lib -lspd -L/usr/local/lib -lode -ldrawstuff\
-lgsl -lgslcblas -lGL -lGLU
INLIBS = 


#SRC = main.cpp terra/terradyn.c
SRC = main.cpp terra/terradyn2.c

#OBJ = main.o terra/terradyn.o
OBJ = main.o terra/terradyn2.o

TARGET = run

#all : $(TARGET)

$(TARGET) : $(OBJ)
	$(CC) $(CFLAGS) $(D_CFLAGS) -o $@ $^ $(INLIBS) $(LDLIBS)

.cpp.o :
	$(CC) $(CFLAGS) $(D_CFLAGS) $(INLIBS) -o $@ -c $<

.c.o :
	$(CC) $(CFLAGS) $(D_CFLAGS) $(INLIBS) -o $@ -c $<

clean :
	-@rm -f $(OBJ) $(TARGET) run.*
ct :
	rm -rf *~
	rm -rf terra/*~
