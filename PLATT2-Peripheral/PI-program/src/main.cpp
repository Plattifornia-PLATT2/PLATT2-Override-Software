#include "ImageProssesing.cpp"



int main() {

    Camera camera;
    //camera.printCameraInfo();
    auto pos = camera.getImagePos();
    std::cout << "Tag ID: " << pos.tag_id << "\n";
    std::cout << "Position (x, y, z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    std::cout << "Reprojection Error: " << pos.reproj_error << "\n";

    return 0;
}