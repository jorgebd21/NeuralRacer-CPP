#include "Simulation.h"
#include <ctime>
#include <cstdlib>

int main(int argc, char* argv[]) {
    srand(time(NULL));

    Simulation sim;
    sim.Run();

    return 0;
}
