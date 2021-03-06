#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H
#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/SpeedControllerGroup.h>
#include <hal/SimDevice.h>
#include <frc/XboxController.h>
#include "ctre/Phoenix.h"
#include "cmath"
#include "frc/smartdashboard/Smartdashboard.h"


using namespace frc;

class SwerveDrive  
{
	private:
		// Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
		// Translation2d m_frontRightLocation{0.381_m, -0.381_m};
		// Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
		// Translation2d m_backRightLocation{-0.381_m, -0.381_m};

		// SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

		// ChassisSpeeds speeds{1_mps, 3_mps, 1.5_rad_per_s};

	
	public:
		WPI_TalonSRX frontLeftDriveMotor = 20;
		WPI_TalonSRX frontLeftAngleMotor = 21;

		WPI_TalonSRX frontRightDriveMotor = 10;
		WPI_TalonSRX frontRightAngleMotor = 11;

		WPI_TalonSRX backLeftDriveMotor = 30;
		WPI_TalonSRX backLeftAngleMotor = 31;

		WPI_TalonSRX backRightDriveMotor = 40;
		WPI_TalonSRX backRightAngleMotor = 41;

		PigeonIMU gyro = (0);

		double robotWidth = 23.25;
		double robotLength = 23.25;
		double z = 0;
		double ypr[3];

		double rightMod = 0;
		double leftMod = 0;

		void drive(double xv, double yv, double omega);
		SwerveDrive();
		~SwerveDrive();

		double frontRightAngle = 0;
		double frontLeftAngle = 0;
    	double backRightAngle = 0;
    	double backLeftAngle = 0;

};
#endif