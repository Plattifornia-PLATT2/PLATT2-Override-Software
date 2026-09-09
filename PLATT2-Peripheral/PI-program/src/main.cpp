#include "imageProssesing.hpp"
#include "uartLink.hpp"

int main() {

    //Camera camera("frontCam");
    //camera.printCameraInfo();
    //auto pos = camera.getImagePos();
    //std::cout << "Tag ID: " << pos.tag_id << "\n";
    //std::cout << "Position (x, y, z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    //std::cout << "Reprojection Error: " << pos.reproj_error << "\n";
    
    UartTextLink link("/dev/ttyAMA0", B115200);

    if (!link.open()) {
    std::cerr << "open failed: " << link.lastError() << "\n";
    return 1;
}


    link.sendLine("hello");

    return 0;

}