# You must install Boost (sudo apt install libboost-all-dev)

SRC_DIR=src/
SRCS=$(wildcard $(SRC_DIR)*.cpp)
OBJ_DIR=obj/
OBJ=$(patsubst $(SRC_DIR)%.cpp,$(OBJ_DIR)%.o,$(SRCS))
INC=-I/usr/include/spinnaker -Iinclude/ -Iinclude/mavlink/v2.0/
# This makes it so that a change to any project-specific header file causes all .o files to rebuild
PROJ_INC=$(wildcard include/*.hpp include/*.h)
OUTPUT=FlightRecorder
OPTIONS=-std=c++11
LIBS=-lSpinnaker -lopencv_core -lopencv_imgcodecs -lopencv_features2d -lopencv_imgproc -lpthread

all: $(OUTPUT)

# -f makes it return success when there's no file to delete
.PHONY: clean
clean: 
	rm -f $(OUTPUT)
	
$(OUTPUT): $(OBJ)
	g++ $^ -o $(OUTPUT) $(INC) $(LIBS)

$(OBJ_DIR)%.o: $(SRC_DIR)%.cpp $(PROJ_INC)
	mkdir -p obj
	g++ -c -o $@ $< $(INC) $(OPTIONS) 
