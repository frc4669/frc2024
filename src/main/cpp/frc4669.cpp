
#include "frc4669.h"
void frc4669::ConfigureMotor(ctre::phoenix6::hardware::TalonFX &motor, bool isInverted) {
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};

    // Motor current configuration (exceeding these limits generally damages the motor)
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 25;
    currentLimitsConfigs.SupplyCurrentThreshold = 25;
    currentLimitsConfigs.SupplyTimeThreshold = 0.5;

    // Neutral mode configuration (sets motor to passively reduce motion without other input)
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Reverses default motor direction (needed if motor is orientated in an undesirable position)
    motorOutputConfigs.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    if (isInverted) {
        motorOutputConfigs.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    } 

    //! safety is enabled

    
    
    motor.GetConfigurator().Apply(talonFXConfigs);
    motor.GetConfigurator().Apply(currentLimitsConfigs);
    motor.GetConfigurator().Apply(motorOutputConfigs);

    // Configures time taken to move motor from neutral to full power
    //   motor.ConfigOpenloopRamp(0.1);
    //   motor.ConfigClosedloopRamp(0);
}
void frc4669::ConfigureRevMotor(rev::CANSparkMax &motor, bool isInverted) {
    // motor.SetSmartCurrentLimit(25);
    
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    motor.SetInverted(isInverted);
}


