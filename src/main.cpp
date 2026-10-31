#include "Simulation.h"
#include <string>
#include <iostream>
#include <filesystem>

using namespace std;

int main(int argc, char* argv[]) {
    // If executed from build/ or build/bin, normalize working directory to project root
    if (!std::filesystem::exists("data")) {
        if (std::filesystem::exists("../data")) {
            std::filesystem::current_path("..");
        } else if (std::filesystem::exists("../../data")) {
            std::filesystem::current_path("../..");
        }
    }

    bool isHeadless = false;
    bool startTraining = false;
    int maxGenerations = -1;

    if (argc > 1) {
        for(int i = 1; i < argc; i++) {
            string arg = argv[i];
            if(arg == "--headless"){
                isHeadless = true;
            }else if(arg == "--training" || arg == "--train"){
                startTraining = true;
            }else if((arg == "--generations" || arg == "-g") && i + 1 < argc){
                maxGenerations = std::stoi(argv[++i]);
            }else if(arg == "--help"){
                cout << "Usage: ./app [--headless] [--training] [--generations <N>]" << endl;
                return 0;
            }else{
                cout << "Unknown argument: " << argv[i] << endl;
                cout << "Usage: ./app [--help] to show arguments" << endl;
                return 1;
            }
        }
    }

    Simulation sim(isHeadless, startTraining, maxGenerations); 
    sim.Run();

    return 0;
}