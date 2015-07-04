CXX = clang++
AR = ar
CXX_FLAGS = -std=c++11 -O3 -Wall -I/usr/local/include -I/usr/include

SRC = matrix.cpp vector.cpp
OBJ = matrix.o vector.o
TARGET = libmatrix.a

all: $(TARGET)

$(TARGET) : $(OBJ)
	$(AR) -r $(TARGET) $(OBJ)

.c.o :
	$(CXX) $(CXX_FLAGS) -o $@ -c $<

clean :
	-@rm *.o $(TARGET)

.PHONY:clean all
