#ifndef COLORSORT_HPP
#define COLORSORT_HPP

// ** PLATT2 Library Includes **
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

// ** PROS API Includes **
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

// ** Standard Library Includes**
#include <atomic>
#include <memory>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for color sorting subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace colorsort{

    /**
     * @brief Enumeration for ball colors.
     * @author Dominic Young
     */
    enum BallColor{
        OFF = 0,
        RED = 1,
        BLUE = 2
    };

    enum SortDirection{
        FRONT = 0,
        REAR = 1
    };

    /**
     * @brief Subsystem to run the color sort process.
     * @author Dominic Young
     */
    class ColorSortSubsystem{
        private:
        std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem;
        std::unique_ptr<pros::Optical> optical_sensor;

        BallColor current_color = OFF;
        SortDirection current_direction = FRONT;

        /**
         * @brief The amount by which the red value must exceed the blue value (or vice versa) for a ball to be considered that color.
         * 
         */
        const double FRONT_LOAD_BALL_REJECTION_THRESHOLD = 0.6;

        /**
         * @brief The amount by which the red value must exceed the blue value (or vice versa) for a ball to be considered that color while using the matchloader.
         * 
         */
        const double MATCHLOAD_BALL_REJECTION_THRESHOLD = 2.0;

        /**
         * @brief Threshold for the optical sensor's proximity reading to determine if a ball is present.
         * 
         */
        const double BALL_DISTANCE_THRESHOLD = 250;

        /**
         * @brief Boolean to track whether color sort is active or not.
         * 
         */
        bool isActive = false;

        /**
         * @brief The task that runs the color sort routine.
         * 
         */
        pros::Task color_sort_task;

        public:
        
        /**
         * @brief Set the Sorted Color
         * 
         * @param color Enumeration to set the current color
         * @param direction Enumeration to set the sorting direction
         */
        void setSortOption(BallColor color);

        /**
         * @brief Get the currently sorted color
         * 
         * @return BallColor The enum for the color being sorted
         */
        BallColor getSortedColor();

        /** @brief Get the current sort direction
         * 
         * @return SortDirection The enum for the current sort direction
         */
        SortDirection getSortDirection();

        /**
         * @brief Check if the color sort is active
         * 
         * @return bool Whether or not the color sort is active
         */
        bool isSortActive();

        /**
         * @brief Step through the color options
         * 
         */
        void toggleSortedColor();
        
        /**
         * @brief Run the color sort process
         * 
         */
        void run();

        /**
         * @brief Construct a new Color Sort Subsystem object
         * 
         * @param intake_subsystem Intake subsystem object
         * @param optical_sensor Optical sensor object
         */
        ColorSortSubsystem(
            std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem,
            std::unique_ptr<pros::Optical> optical_sensor
        );
    };
}
}
}
}

#endif