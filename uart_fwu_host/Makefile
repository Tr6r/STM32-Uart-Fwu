# Makefile đơn giản để compile send_frame.cpp

# Compiler
CXX = g++
# Compiler flags
CXXFLAGS = -Wall -O2 -std=c++17

# Tên file executable
TARGET = host

# Source file
SRC = host.cpp

# Rule mặc định
all: $(TARGET)

# Rule compile
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Rule clean
clean:
	rm -f $(TARGET)
