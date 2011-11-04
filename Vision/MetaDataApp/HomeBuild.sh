 #!/bin/sh

g++ src/*.cpp src/*.h `wx-config --libs --cxxflags` -o GUI -l boost_filesystem 
./GUI

