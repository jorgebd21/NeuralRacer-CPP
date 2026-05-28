#include "Simulation.h"
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {
    bool isHeadless = false;

    if (argc > 1) {
        for(int i = 1; i < argc; i++) {
            string arg = argv[i];
            if(arg == "--headless"){
                    isHeadless = true;
                    break;
            }else if(arg == "--help"){
                    cout << "Usage: ./app [--headless]" << endl;
                    return 0;
            }else{
                    cout << "Unknown argument: " << argv[i] << endl;
                    cout << "Usage: ./app [--help] to show arguments" << endl;
                    return 1;
            }
        }
    }

    Simulation sim(isHeadless); 
    sim.Run();

    return 0;
}