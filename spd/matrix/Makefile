CC = g++
AR = ar
CFLAGS = -O3 -Wall -I/usr/local/include

SRC = matrix.c vector.c
OBJ = matrix.o vector.o
TARGET = libmatrix.a

$(TARGET) : $(OBJ)
	$(AR) -r $(TARGET) $(OBJ)

.c.o :
	$(CC) $(CFLAGS) -o $@ -c $<
clean :
	-@rm *.o $(TARGET)
