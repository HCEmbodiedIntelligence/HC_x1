#include "camera_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <memory>

int main(int argc, char** argv) 
{
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    CustomLogger::Config config;
    config.base_path = "/var/tmp/AGILogs";
    CustomLogger& cl = CustomLogger::instance();
    cl.initialize(config);
    g_logger = cl.getLogger("CameraDriver");

    rclcpp::init(argc, argv);
    
    try 
    {
        g_driver = std::make_shared<CameraDriver>();
        if(!g_driver->initialize()) return -1;
        rclcpp::spin(g_driver);
    } 
    catch (const std::exception& e) 
    {
        g_logger->error("Camera driver exception: {}!\n", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    g_logger->info("Camera driver exited.\n");
    return 0;
}