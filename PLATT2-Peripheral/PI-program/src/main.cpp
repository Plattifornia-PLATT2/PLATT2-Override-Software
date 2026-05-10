#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

int main() {
    // Create tag family
    apriltag_family_t *tf = tag36h11_create();

    // Create detector
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Print success
    std::cout << "AprilTag detector created successfully!" << std::endl;
    std::cout << "Tag family: tag36h11" << std::endl;
    std::cout << "Compilation OK!" << std::endl;

    // Cleanup
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}