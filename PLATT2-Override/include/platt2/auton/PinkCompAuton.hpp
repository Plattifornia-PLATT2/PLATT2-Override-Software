#ifndef PINKCOMPAUTON_HPP
#define PINKCOMPAUTON_HPP

#include "IAuton.hpp"

/**
 * @brief Namespace for all PLATT2 Library Code
 * @authors PLATT2 development team
 */
namespace platt2{

/**
 * @brief Namespace for all autonomous routines
 * @author Dominic Young
 */
namespace auton{

/**
 * @brief The pink robot competition autonomous routine implementation
 * @authors Brett Kucko, Quinn Smith, and Logan Wolf
 */
class PinkCompAuton : public auton::IAuton {
    private:

    const std::string AUTON_NAME {"Pink_Comp_WP"};

    const double STARTING_X_POSITION {56};
    const double STARTING_Y_POSITION {8.5};
    const double STARTING_HEADING {90.0};
    
    public:
    /**
     * @brief Initalizes the autonomous routine()
     * 
     * @param holonomic_subsytem The holonomic subsystem to use
     * @param odometry_subsystem The odometry subsystem to use
     * @param intake_subsystem The intake subsystem to use
     * @param color_sort_subsystem The color sort subsystem to use
     */
    void init() override; 

    std::string getName() const override;

    /**
     * @brief Runs the autonomous routine
     * 
     */
    void run() override;
};

}}

#endif