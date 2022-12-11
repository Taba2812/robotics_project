
#include <sl/Camera.hpp>

using namespace sl;

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Open the camera
    ERROR_CODE returned_state = zed.open();
    if (returned_state != ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // Get camera information (ZED serial number)
    auto camera_infos = zed.getCameraInformation();
    printf("Hello! This is my serial number: %d\n", camera_infos.serial_number);

    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}