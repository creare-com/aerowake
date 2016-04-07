// JDW 2016-2-10
// Copyright Creare 2016
#include <iostream>
#include <string>
#include <getopt.h>
#include <signal.h>
#include "ReelController.hpp"
using namespace std;

bool g_interrupted;

void my_signal_handler(int s) {
    g_interrupted = true;
}

int main(int argc, char* argv[]) {
    string port = "USB0";
    g_interrupted = false;
    
    signal(SIGINT,  my_signal_handler); // ctrl-c
    signal(SIGTERM, my_signal_handler); // killall
    
    // Parse command line options
    opterr = 0;
    char opt ;
    while ( (opt = getopt(argc, argv, "h::p::")) != (char)(-1) )
    {
        switch ( opt ) {
            case 'h':
                cout << "Usage: ./control_test [-p/dev/ttyUSB0]" << endl;
                break;
            case 'p':
            {
                if(optarg == NULL) {
                    cout << "Couldn't parse port (did you put a space after -p?)." << endl;
                } else {
                    port = (string)optarg;
                }
                break;
            }
            case '?':
            default:
                cout << "Unknown option: " << (char)optopt << endl;
                break;
        }
    }
    
    cout << "Starting.  Using port: " << port << endl;
    try {
        ReelController rc(port);
        rc.test(); 
    } catch (exception e) {
        cout << "Caught exception! " << e.what() << endl;
    }
    
    cout << "Done!" << endl;
    
    
    cout << "Waiting for interrupt." << endl;
    while(!g_interrupted);
    cout << "Cleaning up." << endl;
}
