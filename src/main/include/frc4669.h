
#pragma once
#include "Constants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <rev/CANSparkMax.h>

namespace frc4669 {
    void ConfigureMotor(ctre::phoenix6::hardware::TalonFX &motor, bool isInverted); 
    void ConfigureRevMotor(rev::CANSparkMax &motor, bool isInverted);
}