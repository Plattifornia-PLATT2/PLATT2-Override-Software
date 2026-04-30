#ifndef MENU_HPP
#define MENU_HPP

#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/EDriverConfig.hpp"
#include "platt2/robot/Robot.hpp"

#include "liblvgl/lvgl.h"

#include <stdio.h>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development Team.
 */
namespace platt2{

/**
 * @brief Namespace for config menu screen.
 * @author Dominic Young
 */
namespace menu{
    /**
     * @brief A struct for all the necessary config enums
     * @author Dominic Young
     */
    struct FullConfig{
    robot::AutonConfig autonConfig;
    robot::RobotConfig robotConfig;
    robot::DriverProfile driverProfile;
    robot::AllianceConfig allianceColor;
    };

/**
 * @brief A class for the config menu to set the robot configuration
 * @author Dominic Young
 */
class RobotConfigMenu {
public:
    /**
     * @brief Construct a new Robot Config Menu object
     * 
     */
    RobotConfigMenu() = default;

    /**
     * @brief Displays the menu on screen
     * 
     */
    void build();

    /**
     * @brief Get the Robot Config enum state
     * 
     * @return int Enum as an integer
     */
    int getRobotConfig() const { return robot_config; }    // 0 Pink, 1 Purple

    /**
     * @brief Get the Driver Profile enum state
     * 
     * @return int Enum as an integer
     */
    int getDriverProfile() const { return driver_profile; } // 0 Jon, 1 Quinn

    /**
     * @brief Get the Auton Mode enum state
     * 
     * @return int Enum as an integer
     */
    int getAutonMode() const { return auton_mode; }         // 0 Comp, 1 Skills
    
    /**
     * @brief Check if the menu is currently in use
     * 
     * @return true If the menu is in use
     * @return false If the menu is not in use
     */
    bool menuInUse();

    /**
     * @brief Get the Full Config struct with current values
     * 
     * @return FullConfig The full configuration of the robot
     */
    FullConfig getFullConfig();

private:
    // stored values
    robot::RobotConfig robot_config = robot::NO_ROBOT;
    robot::DriverProfile driver_profile = robot::NO_DRIVER;
    robot::AutonConfig auton_mode = robot::NO_AUTON;
    robot::AllianceConfig alliance_color = robot::NO_ALLIANCE;

    // LVGL objects
    lv_obj_t* btn_pink   = nullptr;
    lv_obj_t* btn_purple = nullptr;
    lv_obj_t* btn_jon    = nullptr;
    lv_obj_t* btn_quinn  = nullptr;
    lv_obj_t* btn_red   = nullptr;
    lv_obj_t* btn_blue = nullptr;
    lv_obj_t* btn_save   = nullptr;

    lv_obj_t* btn_pink_comp1 = nullptr;
    lv_obj_t* btn_pink_comp2 = nullptr;
    lv_obj_t* btn_purple_comp1 = nullptr;
    lv_obj_t* btn_purple_comp2 = nullptr;
    lv_obj_t* btn_pink_skills = nullptr;
    lv_obj_t* btn_purple_skills = nullptr;

    // helpers
    void saveConfig();
    void loadConfig();
    void setButtonChecked(lv_obj_t* btn, bool checked, lv_color_t base_color);
    void updateButtonStates();
    void applyColorTheme(lv_obj_t* btn, lv_color_t color);
    void loadStartMenu();

    // static event callbacks (forward to instance via user_data)

    static void cbRobotConfig(lv_event_t* e);
    static void cbDriverProfile(lv_event_t* e);
    static void cbAutonMode(lv_event_t* e);
    static void cbAlliance(lv_event_t* e);
    static void cbSave(lv_event_t* e);

    bool isInUse = true;

};

}}
#endif