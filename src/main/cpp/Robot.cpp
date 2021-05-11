// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "SwerveDrive.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace frc;

void Robot::RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    // gyro.SetYaw(0);
    // a_gyro.Calibrate();


}
void Robot::RobotPeriodic() {

//     double xvv = driveController.GetX(GenericHID::JoystickHand::kLeftHand);
  //     double yvv = driveController.GetY(GenericHID::JoystickHand::kLeftHand);

          frc::SmartDashboard::PutNumber("Shooter Angle", shooterAngle);

          frc::SmartDashboard::PutNumber("Get Shooter Angle angle left", shooterAngleServoLeft.GetAngle());
          frc::SmartDashboard::PutNumber("Get Shooter Angle angle Right", shooterAngleServoRight.GetAngle());

          frc::SmartDashboard::PutNumber("Get Shooter angle speed left", shooterAngleServoLeft.GetSpeed());
          frc::SmartDashboard::PutNumber("Get Shooter angle speed Right", shooterAngleServoRight.GetSpeed());   

         frc::SmartDashboard::PutNumber("shooter motor velocity", shooterMotor.GetSelectedSensorVelocity());





      frc::SmartDashboard::PutNumber("drive controller x", driveController.GetX(GenericHID::JoystickHand::kLeftHand));
      frc::SmartDashboard::PutNumber("drive controller y", driveController.GetY(GenericHID::JoystickHand::kLeftHand));
      frc::SmartDashboard::PutNumber("front Left encoder", frontLeftAngleMotor.GetSelectedSensorPosition());
      frc::SmartDashboard::PutNumber("front Right encoder", frontRightAngleMotor.GetSelectedSensorPosition());
       frc::SmartDashboard::PutNumber("front Left encoder drive", frontLeftDriveMotor.GetSelectedSensorPosition());
      frc::SmartDashboard::PutNumber("front Right encoder drive", frontRightDriveMotor.GetSelectedSensorPosition());
    

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
    //gyro.SetYaw(0);
}

void Robot::AutonomousPeriodic() {
if (frontLeftDriveMotor.GetSelectedSensorPosition() < 50000){ //drive straight
    inputX = 0;
    inputY = -0.3;
     }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 110000){ //drive left
    inputX = -0.3;
    inputY = 0;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 275000){ //drive str
    inputX = 0;
    inputY = -0.3;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 330000){ //drive right
    inputX = 0.3;
    inputY = 0;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 390000){ //drive stright
    inputX = 0;
    inputY = -0.3;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 450000){ //drive 90 left
    inputX = -0.3;
    inputY = 0;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 505000){ //drive back
    inputX = 0;
    inputY = 0.3;
    }
else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 565000){ //drive right
    inputX = 0.3;
    inputY = 0;
    }
// else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 800000){ //drive back
//     inputX = 0;
//     inputY = 1;
//     }
/////////////////////////
// else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 780000){ //drive left
//     inputX = -1;
//     inputY = 0;
//     }
// else if (frontLeftDriveMotor.GetSelectedSensorPosition() < 840000){ //drive back
//     inputX = 0;
//     inputY = 1;
//     }
else { //STOP
inputY = 0;
inputX = 0;
}
driveTrain.drive(inputX, inputY, inputOmega, 1);
 }

void Robot::TeleopInit() {
        //gyro.SetYaw(180);
        //a_gyro.Calibrate();
        //a_gyro.Reset();
table->PutNumber("ledMode",3);
table->PutNumber("camMode",0);

    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    frontRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);


}
void Robot::TeleopPeriodic() {

// frc::SmartDashboard::PutNumber("gyro angle", a_gyro.GetAngle());
// frc::SmartDashboard::PutNumber("gyro offset", a_gyro.GetOffset());

      if (driveController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand) > .1){
        table->PutNumber("ledMode",1);
                speedMul = 0;
        std::cout<<"pressed"<<std::endl;     
   table->PutNumber("camMode",1);

    }else{
        table->PutNumber("ledMode",3);
        table->PutNumber("camMode",0);
speedMul = table->GetNumber("tx",0.0);

        

}
 driveTrain.drive(driveController.GetX(GenericHID::JoystickHand::kLeftHand), driveController.GetY(GenericHID::JoystickHand::kLeftHand), (driveController.GetX(GenericHID::JoystickHand::kRightHand)*0.6), speedMul);
//  driveTrain.drive(LogitechStick.GetX(),LogitechStick.GetY(),(LogitechStick.GetTwist()*0.5),speedMul);   
//FEEDER CONTROL.

    // if (driveController.GetAButton()){
    //   feedPeriodic();
    // }else{
    //   feedON = false;
    //   feedElpasedTime = 0;
    //   prevFeedTime = time.Get();
    //   feederMotor.Set(ControlMode::PercentOutput,0.0);
    //  }

    if(driveController.GetBackButtonReleased()){
        //gyro.SetYaw(180);
    }

    if (driveController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand) > .1 && shooterMotor.GetSelectedSensorVelocity() > 16500){
        feederMotor.Set(ControlMode::PercentOutput,0.5);

    } else if (driveController.GetBumper(GenericHID::JoystickHand::kLeftHand) == true){
        feederMotor.Set(ControlMode::PercentOutput,-0.5);

    } else {
        feederMotor.Set(ControlMode::PercentOutput,0);


    }
    if (driveController.GetYButton())
    {
        double feederEncoder = feederMotor.GetSelectedSensorPosition() + 2048;
        while (feederMotor.GetSelectedSensorPosition() < feederEncoder)
        {        feederMotor.Set(ControlMode::PercentOutput,0.5);
}
        feederMotor.Set(ControlMode::PercentOutput,0);

        
    }
    
//

 //INTAKE CONTROL.
        if(controlController.GetYButton()){
                intakeMotor.Set(ControlMode::PercentOutput,-1);

        } else if (controlController.GetXButton())	{
            intakeMotor.Set(ControlMode::PercentOutput,0);

        }

        if(controlController.GetAButton()){
            intakeLiftMotor.Set(ControlMode::PercentOutput,0.2);

        }else if (controlController.GetBButton()){
            intakeLiftMotor.Set(ControlMode::PercentOutput,-0.2);

        }else{
            intakeLiftMotor.Set(ControlMode::PercentOutput,0);

        }
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
    
        
        
//


//SHOOTER MOTOR CONTROL.
 
     manualAngleControl();

    if (driveController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand) > .1 ){
        // shooterMotor.Set(ControlMode::PercentOutput,driveController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand));
                shooterMotor.Set(ControlMode::PercentOutput,.9);

        // shooterMotor.Set(shooterValue);

        // std::cout<<"SHOOTER MOTOR ON"<<std::endl;

    } else if (driveController.GetBumper(GenericHID::JoystickHand::kRightHand) == true){
        shooterMotor.Set(ControlMode::PercentOutput,-.5);
        // std::cout<<"SHOOTER MOTOR REVERSE"<<std::endl;

    } else{
        shooterMotor.Set(ControlMode::PercentOutput, 0);
        // std::cout<<"SHOOTER MOTOR OFF"<<std::endl;

    }
//



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

void Robot::manualAngleControl(){
    // if (arcade.GetRawButton(12)){
    //     shooterAngle = shooterAngle + 0.001;

    // }else if(arcade.GetRawButton(11)){
    //     shooterAngle = shooterAngle - 0.001;
    // } 
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