CXX = clang++
AR = ar
CXX_FLAGS = -std=c++11 -O3 -Wall -I/usr/local/include

# SRC = matrix.cpp vector.cpp
SRC = vector.cpp
# OBJ = matrix.o vector.o
OBJ = vector.o
TARGET = libmatrix.a

all: $(TARGET)

$(TARGET) : $(OBJ)
	$(AR) -r $(TARGET) $(OBJ)

.c.o :
	$(CXX) $(CXX_FLAGS) -o $@ -c $<

clean :
	-@rm *.o $(TARGET)

.PHONY:clean all
