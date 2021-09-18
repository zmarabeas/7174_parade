// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace frc;

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }
void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }
void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() {
    //drive
    drivetrain.ArcadeDrive(driveController.GetY(GenericHID::JoystickHand::kLeftHand), 
        driveController.GetX(GenericHID::JoystickHand::kRightHand))
    
    //INTAKE CONTROL.
    // if(controlController.GetYButton()){
    //         intakeMotor.Set(ControlMode::PercentOutput,-1);
    // }else if (controlController.GetXButton())	{
    //     intakeMotor.Set(ControlMode::PercentOutput,0);
    // }

    // if(controlController.GetAButton()){
    //     intakeLiftMotor.Set(ControlMode::PercentOutput,0.2);
    // }else if (controlController.GetBButton()){
    //     intakeLiftMotor.Set(ControlMode::PercentOutput,-0.2);
    // }else{
    //     intakeLiftMotor.Set(ControlMode::PercentOutput,0);
    // }


    manualAngleControl();
    //Angle Motor
    if (controlController.GetBButton()) {
        shooterAngle = 0.456;
        shooterValue = 0.725;
    }else if (controlController.GetAButton()){
        shooterAngle = 0.3600;
        shooterValue = 1;
    }else if (controlController.GetXButton()){
        shooterAngle = 0.33300;//.342
        shooterValue = 1;
    }else{
        shooterAngleServoLeft.Set(shooterAngle);
        shooterAngleServoRight.Set(shooterAngle);
    }

    // feeder
    if (driveController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand) > .1 && shooterMotor.GetSelectedSensorVelocity() > 16500){
        feederMotor.Set(ControlMode::PercentOutput,0.5);

    } else if (driveController.GetBumper(GenericHID::JoystickHand::kLeftHand) == true){
        feederMotor.Set(ControlMode::PercentOutput,-0.5);

    } else {
        feederMotor.Set(ControlMode::PercentOutput,0);
    }

    if (driveController.GetYButton()){
        double feederEncoder = feederMotor.GetSelectedSensorPosition() + 2048;
        while (feederMotor.GetSelectedSensorPosition() < feederEncoder){
            feederMotor.Set(ControlMode::PercentOutput,0.5);
        }
        feederMotor.Set(ControlMode::PercentOutput,0);
    }
    // shooter
    if (driveController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand) > .1 ){
        shooterMotor.Set(ControlMode::PercentOutput,.9);
    } else if (driveController.GetBumper(GenericHID::JoystickHand::kRightHand) == true){
        shooterMotor.Set(ControlMode::PercentOutput,-.5);
    } else{
        shooterMotor.Set(ControlMode::PercentOutput, 0);
    }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::manualAngleControl(){
    if (controlController.GetY(GenericHID::JoystickHand::kLeftHand) < -0.1){
        shooterAngle = shooterAngle + 0.003;
    }else if(controlController.GetY(GenericHID::JoystickHand::kLeftHand) > 0.1){
        shooterAngle = shooterAngle - 0.003;
    } 
    shooterAngle = std::clamp(shooterAngle,-1.0, 1.0);
    shooterAngleServoLeft.Set(shooterAngle);
    shooterAngleServoRight.Set(shooterAngle);
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif