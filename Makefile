TARGET := RRT

CC := g++

# CFLAGS := -O3 -Wall -ffast-math -mavx2 -Rpass=loop-vectorize -Rpass-missed=loop-vectorize -Rpass-analysis=loop-vectorize
CFLAGS := -O3 -Wall

SRC := RRT.cpp Util.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $@

clean:
	$(RM) $(TARGET)
