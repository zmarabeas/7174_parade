// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace frc;

void Robot::RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


}
void Robot::RobotPeriodic() {

//     double xvv = driveController.GetX(GenericHID::JoystickHand::kLeftHand);
  //     double yvv = driveController.GetY(GenericHID::JoystickHand::kLeftHand);


      frc::SmartDashboard::PutNumber("drive controller x", driveController.GetX(GenericHID::JoystickHand::kLeftHand));
      frc::SmartDashboard::PutNumber("drive controller y", driveController.GetY(GenericHID::JoystickHand::kLeftHand));
      frc::SmartDashboard::PutNumber("front Left encoder", frontLeftDriveMotor.GetSelectedSensorPosition());
      frc::SmartDashboard::PutNumber("front Right encoder", frontRightDriveMotor.GetSelectedSensorPosition());
    

  //     driveSpeedd = sqrt((xvv) * (xvv) + (yvv) * (yvv));  
    
  //     if (driveController.GetX(GenericHID::JoystickHand::kLeftHand) < 0){
  //           driveAngle = (driveController.GetX(GenericHID::JoystickHand::kLeftHand)) * 90;

  //     } else {
  //           driveAngle = (-driveController.GetX(GenericHID::JoystickHand::kLeftHand)) * 90;

  //     }
    //  sensorPos = frontRightDriveMotor.GetSelectedSensorPosition();
 }

void Robot::AutonomousInit() {
    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontRightDriveMotor.SetSelectedSensorPosition(0);
    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);
        shooterMotor.Set(ControlMode::PercentOutput,0);

    

}
void Robot::AutonomousPeriodic() {

    // shooterAngleServoLeft.SetSpeed(-.67);
    // shooterAngleServoRight.SetSpeed(-.67);
    //   if (frontLeftDriveMotor.GetSelectedSensorPosition() && frontRightDriveMotor.GetSelectedSensorPosition() && backRightDriveMotor.GetSelectedSensorPosition() && backLeftDriveMotor.GetSelectedSensorPosition() < -100000){
    //   frontLeftDriveMotor.Set(ControlMode::PercentOutput,-0.1);
    //   frontRightDriveMotor.Set(ControlMode::PercentOutput,0.1);
    //   backLeftDriveMotor.Set(ControlMode::PercentOutput,-0.1);
    //   backRightDriveMotor.Set(ControlMode::PercentOutput,0.1);
    // }else
    // {
    //   frontLeftDriveMotor.Set(ControlMode::PercentOutput,0);
    //   frontRightDriveMotor.Set(ControlMode::PercentOutput,0);
    //   backLeftDriveMotor.Set(ControlMode::PercentOutput,0);
    //   backRightDriveMotor.Set(ControlMode::PercentOutput,0);
    // }
    // if(sensorPos < 40000) {
    //   frontLeftDriveMotor.Set(ControlMode::PercentOutput,-0.2);
    //   frontRightDriveMotor.Set(ControlMode::PercentOutput,0.2);
    //   backLeftDriveMotor.Set(ControlMode::PercentOutput,-0.2);
    //   backRightDriveMotor.Set(ControlMode::PercentOutput,0.2);
    //     feederMotor.Set(ControlMode::PercentOutput,0);
    //   shooterMotor.Set(ControlMode::PercentOutput,0);

    // }else
    // {
    //   frontLeftDriveMotor.Set(ControlMode::PercentOutput,0);
    //   frontRightDriveMotor.Set(ControlMode::PercentOutput,0);
    //   backLeftDriveMotor.Set(ControlMode::PercentOutput,0);
    //   backRightDriveMotor.Set(ControlMode::PercentOutput,0);  
    //   feederMotor.Set(ControlMode::PercentOutput,0);
    //   shooterMotor.Set(ControlMode::PercentOutput,0);


      // }
      //     feederMotor.Set(ControlMode::PercentOutput,.5);
      // shooterMotor.Set(ControlMode::PercentOutput,1);

}


void Robot::TeleopInit() {
    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);


}
void Robot::TeleopPeriodic() {
 driveTrain.drive(driveController.GetX(GenericHID::JoystickHand::kLeftHand), driveController.GetY(GenericHID::JoystickHand::kLeftHand), driveController.GetX(GenericHID::JoystickHand::kRightHand));
    
    // if (controlController.GetY(GenericHID::JoystickHand::kLeftHand) < -0.1){
    //     shooterAngle = shooterAngle + 0.003;
    // }else if(controlController.GetY(GenericHID::JoystickHand::kLeftHand) > 0.1){
    //     shooterAngle = shooterAngle - 0.003;
    // } 
    // shooterAngle = std::clamp(shooterAngle,-1.0, 1.0);
    // shooterAngleServoLeft.SetSpeed(shooterAngle);
    // shooterAngleServoRight.SetSpeed(shooterAngle);
    // std::cout << shooterAngle << std::endl;

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
