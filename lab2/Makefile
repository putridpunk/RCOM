CC      ?= cc
CFLAGS  ?= -Wall -Wextra -O2 -std=c99
TARGET   = download
SOURCES  = download.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $(SOURCES)

clean:
	rm -f $(TARGET) *.o
