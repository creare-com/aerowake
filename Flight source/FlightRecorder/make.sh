FILES="FlightRecorder.cpp benchmarker.cpp"
INC="-I/usr/include/spinnaker"
OUTPUT="FlightRecorder"
OPTIONS="-std=c++11"
LIBS="-lSpinnaker -lopencv_core -lopencv_imgcodecs -lopencv_features2d -lopencv_imgproc"

g++ $FILES -o $OUTPUT $INC $LIBS $OPTIONS
