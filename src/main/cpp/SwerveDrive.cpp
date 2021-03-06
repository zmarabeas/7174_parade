#include "SwerveDrive.hpp"  


SwerveDrive::SwerveDrive(){
        
}
	
SwerveDrive::~SwerveDrive()
{
	
}

void SwerveDrive::drive(double xv,double yv,double omega){
    // ChassisSpeeds speeds;
  frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * 0.015;
  frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * 0.015;
  backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * 0.015;
  backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * 0.015;
    double r = sqrt ((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

    double a = xv - omega * (robotLength / r);
    double b = xv + omega * (robotLength / r);
    double c = yv - omega * (robotWidth / r);
    double d = yv + omega * (robotWidth / r);

    //MOTOR SPEED CALCULATIONS
    double backRightSpeed = sqrt ((a * a) + (d * d));
    double backLeftSpeed = sqrt ((a * a)) + ((c * c));
    double frontRightSpeed = sqrt ((b * b)) + ((d * d));
    double frontLeftSpeed = sqrt ((b * b)) + ((c * c));

    //MOTOR ANGLE CALCULATIONS
    double backRightAngleCalc = 180 * atan2 (a, d) / (2*acos(0));
    double backLeftAngleCalc = 180 * atan2 (a, c) / (2*acos(0));
    double frontRightAngleCalc = 180 * atan2 (b, d) / (2*acos(0));
    double frontLeftAngleCalc = 180 * atan2 (b, c) / (2*acos(0));
    //READING GRYO
        gyro.GetYawPitchRoll(ypr);
        z = ypr[0];


frc::SmartDashboard::PutNumber("xv", xv);
frc::SmartDashboard::PutNumber("yv", yv);
frc::SmartDashboard::PutNumber("backRightSpeed", backRightSpeed);
frc::SmartDashboard::PutNumber("backLeftspeed", backLeftSpeed);
frc::SmartDashboard::PutNumber("front right speed", frontRightSpeed);
frc::SmartDashboard::PutNumber("front left speed", frontLeftSpeed);
frc::SmartDashboard::PutNumber("gyro", z);

frc::SmartDashboard::PutNumber("backRightAngle", backRightAngleCalc);
frc::SmartDashboard::PutNumber("backLeftAngleCalc", backLeftAngleCalc);
frc::SmartDashboard::PutNumber("front right AngleCalc", frontRightAngleCalc);
frc::SmartDashboard::PutNumber("front left AngleCalc", frontLeftAngleCalc);
    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (frontLeftSpeed - leftMod));
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (frontRightSpeed - rightMod));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed - rightMod));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backLeftSpeed - leftMod));

//  double robotHeading = (z * M_PI) / 180;
// double temp = yv * Math.cos(gyro) + xv * Math.sin(gyro); 
// xv = -yv * Math.sin(gyro) + xv * Math.cos(gyro); 
// yv = temp; 
//remove  mods if does not work
    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (frontLeftAngleCalc));

        if (frontLeftAngle < frontLeftAngleCalc - 5) {
                frontLeftAngleMotor.Set(ControlMode::PercentOutput,0.1);

        }else if (frontLeftAngle > frontLeftAngleCalc + 5){
                frontLeftAngleMotor.Set(ControlMode::PercentOutput,-0.1);

        }else {
                 frontLeftAngleMotor.Set(ControlMode::PercentOutput,0);

        }

    frontRightAngleMotor.Set(ControlMode::PercentOutput, (frontRightAngleCalc));

        if (frontRightAngle < frontRightAngleCalc - 5) {
                frontRightAngleMotor.Set(ControlMode::PercentOutput,0.1);

        }else if (frontRightAngle > frontRightAngleCalc + 5){
                frontRightAngleMotor.Set(ControlMode::PercentOutput,-0.1);

        }else {
                 frontRightAngleMotor.Set(ControlMode::PercentOutput,0);

        }


    backRightAngleMotor.Set(ControlMode::PercentOutput, (backRightAngleCalc));

        if (backRightAngle < backRightAngleCalc - 5) {
                backRightAngleMotor.Set(ControlMode::PercentOutput,0.1);

        }else if (backRightAngle > backRightAngleCalc + 5){
                backRightAngleMotor.Set(ControlMode::PercentOutput,-0.1);

        }else {
                 backRightAngleMotor.Set(ControlMode::PercentOutput,0);

        }

    backLeftAngleMotor.Set(ControlMode::PercentOutput, (backLeftAngleCalc));

        if (backLeftAngle < backLeftAngleCalc - 5) {
                backLeftAngleMotor.Set(ControlMode::PercentOutput,0.1);

        }else if (backLeftAngle > backLeftAngleCalc + 5){
                backLeftAngleMotor.Set(ControlMode::PercentOutput,-0.1);

        }else {
                 backLeftAngleMotor.Set(ControlMode::PercentOutput,0);

        }


//  comment out if does not work


//     if (z /* - backRightAngleCalc*/ < 0){
//         rightMod = 1/90 * -z;
//         leftMod  = 0;

//     } else if (z /*- backRightAngleCalc*/ > 0) {
//        rightMod = 0;
//        leftMod = 1/90 * z;

//     } else{
//         rightMod = 0;
//         leftMod = 0;
//     }




}