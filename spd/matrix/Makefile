CC = clang++
AR = ar
CFLAGS = -O3 -Wall -std=c++11 -I/usr/local/include

SRC = matrix.cpp vector.cpp
OBJ = matrix.o vector.o
TARGET = libmatrix.a

$(TARGET) : $(OBJ)
	$(AR) -r $(TARGET) $(OBJ)

.c.o :
	$(CC) $(CFLAGS) -o $@ -c $<
clean :
	-@rm *.o $(TARGET)
