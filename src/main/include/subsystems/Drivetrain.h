// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <Constants.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <frc/SPI.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/length.h>
#include <units/angle.h>
#include <AHRS.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>

//setup 4 motors for drive 
// 

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
    
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void CurvatureDrive(double forward, double rotation);
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  frc2::CommandPtr DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation);
  frc2::CommandPtr AutonomousCommand();

  void DriveChassisSpeed(frc::ChassisSpeeds speeds);
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  units::meter_t GetLeftDistance();
  units::meter_t GetRightDistance();
  units::degree_t GetYaw();
  units::degree_t GetPitch();
  frc::Pose2d OdometryPose();

  void ResetOdometry(frc::Pose2d pose);
  void ResetEncoders();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX leftMainMotor { CAN::kLeftMainMotor };
  ctre::phoenix6::hardware::TalonFX leftSecondaryMotor { CAN::kLeftSecondaryMotor };
  ctre::phoenix6::hardware::TalonFX rightMainMotor { CAN::kRightMainMotor };
  ctre::phoenix6::hardware::TalonFX rightSecondaryMotor { CAN::kRightSecondaryMotor };

  ctre::phoenix6::controls::Follower leftFollower { CAN::kLeftMainMotor, true }; // invterted cuz reasons
  ctre::phoenix6::controls::Follower rightFollower { CAN::kRightMainMotor, false};

  frc::DifferentialDrive drive { leftMainMotor, rightMainMotor };

  AHRS m_IMU { frc::SPI::Port::kMXP };
  units::degree_t m_yawOffset = 0_deg;

  frc::DifferentialDriveOdometry m_odometry { frc::Rotation2d(), 0.0_m, 0.0_m, frc::Pose2d() };

  frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };
  frc::SimpleMotorFeedforward<units::meters> m_feedforward { DriveConstants::ks, DriveConstants::kv, DriveConstants::ka };

  frc::PIDController m_leftController { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };
  frc::PIDController m_rightController { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };

  units::second_t m_prevTime;

  frc::Timer m_timer;

  void ConfigureRamsete();

  bool motorTurnInPlace = true;

  frc::Rotation2d rotation;
};
