 #!/bin/sh

g++ src/Main.cpp src/ConverterGUI.h src/ConverterGUI.cpp src/ConverterEvents.cpp src/Converter.cpp src/ChoiceFrame.h src/ChoiceFrame.cpp src/Choice.cpp `wx-config --libs --cxxflags` -o GUI -l boost_filesystem 
./GUI

