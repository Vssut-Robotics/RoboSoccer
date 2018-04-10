#!/bin/bash

# just change the compilation file name

#script for defence pid
g++ -I Headers/ Defense_and_PID.cpp Headers/grSim_Packet.pb.cc Headers/grSim_Commands.pb.cc Headers/grSim_Replacement.pb.cc Headers/robocup_ssl_client.cpp Headers/messages_robocup_ssl_wrapper.pb.cc Headers/messages_robocup_ssl_geometry.pb.cc Headers/messages_robocup_ssl_detection.pb.cc Headers/netraw.cpp -lboost_system -lboost_thread -lprotobuf -pthread

## todo: finalize the compilation script with variables and command line arguments

echo "Done Compiling."
