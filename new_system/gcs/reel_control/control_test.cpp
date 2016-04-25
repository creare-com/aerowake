// JDW 2016-2-10
// Copyright Creare 2016
#include <iostream>
#include <string>
#include <getopt.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include "ReelController.hpp"
using namespace std;

bool g_interrupted;

void my_signal_handler(int s) {
    g_interrupted = true;
}

int main(int argc, char* argv[]) {
    string port = "USB0";
    int cm_to_retract = 10;
    g_interrupted = false;
    
    signal(SIGINT,  my_signal_handler); // ctrl-c
    signal(SIGTERM, my_signal_handler); // killall
    
    // Parse command line options
    opterr = 0;
    char opt ;
    while ( (opt = getopt(argc, argv, "h::p::l::")) != (char)(-1) )
    {
        switch ( opt ) {
            case 'h':
                cout << "Usage: ./control_test [-pUSB0] [-l10]" << endl;
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
            case 'l':
            {
                if(optarg == NULL) {
                    cout << "Couldn't parse length (did you put a space after -l?)." << endl;
                } else {
                    string len = (string)optarg;
                    cm_to_retract = atoi(len.c_str());
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
        double len = rc.getTetherLength();
        cout << "Paid out length = " << len << "m" << endl;
        len += cm_to_retract / 100.0;
        cout << "Retracting to " << len << "m" << endl;
        rc.setMaxTetherSpeed(100);
        rc.setTetherLength(len);
        cout << "Done!" << endl;
        
        cout << "Waiting for interrupt." << endl;
        while(!g_interrupted) {
            double len = rc.getTetherLength();
            cout << "Tether length = " << len << endl;
            sleep(1);
        }
        cout << "Cleaning up." << endl;
    } catch (exception e) {
        cout << "Caught exception! " << e.what() << endl;
    }
}
