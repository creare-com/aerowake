// JDW 2016-2-10
// Copyright Creare 2016
#include <iostream>
#include <string>
#include <getopt.h>
#include "ReelController.hpp"
using namespace std;

int main(int argc, char* argv[]) {
    string port = "/dev/ttyS0";
    
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
    
    cout << "Starting" << endl;
    ReelController rc(port);
    cout << "Done!" << endl;
}
