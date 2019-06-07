# You must install Boost (sudo apt install libboost-all-dev)

SRCS=$(wildcard src/*.cpp)
INC=-I/usr/include/spinnaker -Iinclude/ -Imavlink/include/mavlink/v2.0
PROJ_INC=$(wildcard include/*.hpp include/*.h)
OUTPUT=FlightRecorder
OPTIONS=-std=c++11
LIBS=-lSpinnaker -lopencv_core -lopencv_imgcodecs -lopencv_features2d -lopencv_imgproc -lpthread

all: $(OUTPUT)

# -f makes it return success when there's no file to delete
clean: 
	rm -f $(OUTPUT)
	
$(OUTPUT): $(SRCS) $(PROJ_INC)
	g++ $(SRCS) -o $(OUTPUT) $(INC) $(LIBS) $(OPTIONS)
