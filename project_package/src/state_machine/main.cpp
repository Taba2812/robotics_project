#include "process.h"
#include "concrete_states.h"
#include <unistd.h>

int main(int argc, char **argv){
    Process process;

    for(int i=0; i<4; i++){
        process.execute();
        sleep(1);
    }

    return 0;
}