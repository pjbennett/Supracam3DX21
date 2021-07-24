SRC = 3dx21.c
TARGET = 3dx21

default: $(TARGET)

$(TARGET): 3dx21.c
	gcc -Wall -DUSE_SOCKET -o $(TARGET) $(SRC) `pkg-config --cflags --libs gtk+-3.0` -rdynamic -lm -pthread

debug: 3dx21.c
	gcc -Wall -DUSE_SOCKET -g -O0 -o $(TARGET) $(SRC) `pkg-config --cflags --libs gtk+-3.0` -rdynamic -lm -pthread

clean:
	rm -f $(TARGET)
