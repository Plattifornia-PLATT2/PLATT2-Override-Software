#include "imageProssesing.hpp"
#include "uartPort.hpp"
#include "piLink.hpp"
#include "sharedData.hpp"
#include <thread>
#include <vector>




int main() {

    std::vector<std::jthread> tasks;
    sharedData shared;

    tasks.emplace_back(piLink,std::ref(shared));


    
    return 0;
}


//    UartPort link("/dev/ttyAMA0", B115200);

    //link.sendLine("hello");
    