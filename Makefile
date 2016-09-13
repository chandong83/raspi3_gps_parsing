INC= -I ./include/

OBJ = main.o \
	  gps.o

TARGET = gps_test

$(TARGET): $(OBJ)
	gcc -o $(TARGET) $(OBJ) -lpthread -lm

%.o: %.c
	gcc $(INC) -Wall -c $^ -o $@
.PHONY: clean
clean:
	rm -f $(OBJDIR)/$(TARGET) $(OBJ)
