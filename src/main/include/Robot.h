// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
// #define PRACTICE
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/Servo.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/I2C.h>

std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

using namespace frc;


#define FRONT_LEFT_MOTOR 3
#define BACK_LEFT_MOTOR 6
#define FRONT_RIGHT_MOTOR 1
#define BACK_RIGHT_MOTOR 2

#define FEEDER_MOTOR 9
#define SHOOTER_MOTOR 15

#define ANGLE_SERVO_LEFT 5
#define ANGLE_SERVO_RIGHT 6

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;

        void AutonomousInit() override;
        void AutonomousPeriodic() override;

        void TeleopInit() override;
        void TeleopPeriodic() override;

        void DisabledInit() override;
        void DisabledPeriodic() override;

        void TestInit() override;
        void TestPeriodic() override;

        void manualAngleControl();


    private:
        frc::SendableChooser<std::string> m_chooser;
        const std::string kAutoNameDefault = "Default";
        const std::string kAutoNameCustom = "My Auto";
        std::string m_autoSelected;
        PowerDistributionPanel PDP{0};

        XboxController driveController{0};
        XboxController controlController{1};
        Joystick LogitechStick{2};
        
        double speedMul = 0;


        // drivetrain
        WPI_TalonSRX frontLeftMotor{FRONT_LEFT_MOTOR}; 
        WPI_TalonSRX backLeftMotor{BACK_LEFT_MOTOR};  
        SpeedControllerGroup leftSideMotors{frontLeftMotor, backLeftMotor};
        WPI_TalonSRX frontRightMotor{FRONT_RIGHT_MOTOR};  
        WPI_TalonSRX backRightMotor{BACK_RIGHT_MOTOR}; 
        SpeedControllerGroup rightSideMotors{frontRightMotor, backRightMotor};

        DifferentialDrive drivetrain{leftSideMotors, rightSideMotors};

        //ACTIONARY MOTORS
        WPI_TalonSRX shooterMotor{SHOOTER_MOTOR}; 
        WPI_TalonSRX feederMotor{FEEDER_MOTOR}; 
        Servo shooterAngleServoLeft {ANGLE_SERVO_LEFT};
        Servo shooterAngleServoRight {ANGLE_SERVO_RIGHT};

        /* not in use */
        // WPI_TalonSRX intakeMotor{8};//9
        // WPI_TalonSRX intakeLiftMotor{100};//10
        // WPI_TalonSRX wheelOfFortuneMotor {11};

        //DRIVETRAIN STATES
        bool aunkrModeState = false;
        bool rawModeState = false;
        bool climbModeState = false;
        bool ballTrackState = false;
        double driveSpeed = 0;
        double turnSpeed = 0;

        Timer time;
        double buttonPressedTime = 0;
        double prevFeedTime = 0.0;
        double feedOffTime = 1.0;
        double feedOnTime = 1.0;
        double feedElpasedTime = 0.0;
        bool feedON = false;
        
        uint8_t buffer = 0;
        I2C arduino {I2C::Port::kOnboard, 0x03};

        double shooterAngle = 0.345000;//manual control of servo//.7 angle servo init line inner port
        double shooterAngleTrench = -0.67;//.68 angle servo trench

        double sensorPos = 0;
        double driveSpeedd = 0;
        double driveAngle = 0;

        double shooterValue = 0;
};