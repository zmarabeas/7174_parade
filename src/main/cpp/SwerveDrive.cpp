#include "SwerveDrive.hpp"  


SwerveDrive::SwerveDrive(){
        
}
	
SwerveDrive::~SwerveDrive()
{
	
}

void SwerveDrive::drive(double xv,double yv,double omega,double speedMul){
    // ChassisSpeeds speeds;
  frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
  backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * 0.0146484375;
    double r = sqrt ((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

if (yv < 0.075 && yv > -0.075){
yv = 0;
}
if (xv < 0.075 && xv > -0.075){
xv = 0;
}
if (omega < 0.075 && omega > -0.075){
omega = 0;

        gyroPID.EnableContinuousInput(-180, 180);
        gyroMul = gyroPID.Calculate(speedMul, 0);
}
        //gyro.GetYawPitchRoll(ypr);
        z = 180;//ypr[0];
        robotHeading = 2*acos(0);//(-ypr[0] * 2*acos(0)) / 180;

    // xv = (xv)+0.03;//.00975
    // yv = yv;

double temp = yv * cos(robotHeading) + xv * sin(robotHeading); 
xv = -yv * sin(robotHeading) + xv * cos(robotHeading); 
yv = temp; 
    omega = omega - gyroMul;

    double a = xv - omega * (robotLength / r);
    double b = xv + omega * (robotLength / r);
    double c = yv - omega * (robotWidth / r);
    double d = yv + omega * (robotWidth / r);
    
    //MOTOR SPEED CALCULATIONS
     backRightSpeed = sqrt ((a * a) + (d * d));
     backLeftSpeed = sqrt ((a * a)) + ((c * c));
     frontRightSpeed = sqrt ((b * b)) + ((d * d));
     frontLeftSpeed = sqrt ((b * b)) + ((c * c));

    //MOTOR ANGLE CALCULATIONS
     backLeftAngleCalc = 180 * atan2 (a, d) / (2*acos(0));
     backRightAngleCalc = 180 * atan2 (a, c) / (2*acos(0));
     frontLeftAngleCalc = 180 * atan2 (b, d) / (2*acos(0));
     frontRightAngleCalc = 180 * atan2 (b, c) / (2*acos(0));

     
// if(yv >= 0){

//            backRightSpeed = backRightSpeed;
//      backLeftSpeed = backLeftSpeed;
//      frontRightSpeed = frontRightSpeed;
//      frontLeftSpeed = frontLeftSpeed;

//     //MOTOR ANGLE CALCULATIONS
//      backLeftAngleCalc = 180 * atan2 (a, d) / (2*acos(0));
//      backRightAngleCalc = 180 * atan2 (a, c) / (2*acos(0));
//      frontLeftAngleCalc = 180 * atan2 (b, d) / (2*acos(0));
//      frontRightAngleCalc = 180 * atan2 (b, c) / (2*acos(0));
// }else{
//      backRightSpeed = -backRightSpeed;
//      backLeftSpeed = -backLeftSpeed;
//      frontRightSpeed = -frontRightSpeed;
//      frontLeftSpeed = -frontLeftSpeed;
// }

//     //MOTOR ANGLE CALCULATIONS
//     if (xv > 0.075 && yv < -0.075){
//      backLeftAngleCalc = (180 * atan2 (a, d) / (2*acos(0))) - 180;
//      backRightAngleCalc = (180 * atan2 (a, c) / (2*acos(0))) - 180;
//      frontLeftAngleCalc = (180 * atan2 (b, d) / (2*acos(0))) - 180;
//      frontRightAngleCalc = (180 * atan2 (b, c) / (2*acos(0))) - 180;    

//      }else if (xv < 0.075 && yv < -0.075){
//      backLeftAngleCalc = (180 * atan2 (a, d) / (2*acos(0))) + 180;
//      backRightAngleCalc = (180 * atan2 (a, c) / (2*acos(0))) + 180;
//      frontLeftAngleCalc = (180 * atan2 (b, d) / (2*acos(0))) + 180;
//      frontRightAngleCalc = (180 * atan2 (b, c) / (2*acos(0))) + 180;           
// }
     



frc::SmartDashboard::PutNumber("xv", xv);
frc::SmartDashboard::PutNumber("yv", yv);
frc::SmartDashboard::PutNumber("backRightSpeed", backRightSpeed);
frc::SmartDashboard::PutNumber("backLeftspeed", backLeftSpeed);
frc::SmartDashboard::PutNumber("front right speed", frontRightSpeed);
frc::SmartDashboard::PutNumber("front left speed", frontLeftSpeed);
frc::SmartDashboard::PutNumber("gyro", z);
frc::SmartDashboard::PutNumber("head", robotHeading);
frc::SmartDashboard::PutNumber("speedMul", speedMul);



frc::SmartDashboard::PutNumber("backRightAngle", backRightAngleCalc);
frc::SmartDashboard::PutNumber("backLeftAngleCalc", backLeftAngleCalc);
frc::SmartDashboard::PutNumber("front right AngleCalc", frontRightAngleCalc);
frc::SmartDashboard::PutNumber("front left AngleCalc", frontLeftAngleCalc);

frc::SmartDashboard::PutNumber("backRightAngle", backRightAngle);
frc::SmartDashboard::PutNumber("backLeftAngle", backLeftAngle);
frc::SmartDashboard::PutNumber("front right Angle", frontRightAngle);
frc::SmartDashboard::PutNumber("front left Angle", frontLeftAngle);




frc::SmartDashboard::PutNumber("gyroMul", gyroMul);

    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (frontLeftSpeed * 0.6));//////////////////////////////////////////////////////////speed * 0.6 multi
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (frontRightSpeed * 0.6));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * 0.6));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backLeftSpeed * 0.6));
        // steerPID.SetIntegratorRange(-0.02, 0.02);
        steerPID.EnableContinuousInput(-180, 180);
        drivePID.EnableContinuousInput(-1, 1);

//remove  mods if does not work
    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontLeftAngle, frontLeftAngleCalc)));

        // if (frontLeftAngle < frontLeftAngleCalc - 7.5) {
        //         frontLeftAngleMotor.Set(ControlMode::PercentOutput,0.2);

        // }else if (frontLeftAngle > frontLeftAngleCalc + 7.5){
        //         frontLeftAngleMotor.Set(ControlMode::PercentOutput,-0.2);

        // }else {
        //          frontLeftAngleMotor.Set(ControlMode::PercentOutput,0);

        // }

    frontRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontRightAngle,frontRightAngleCalc)));

        // if (frontRightAngle < frontRightAngleCalc - 7.5) {
        //         frontRightAngleMotor.Set(ControlMode::PercentOutput,0.2);

        // }else if (frontRightAngle > frontRightAngleCalc + 7.5){
        //         frontRightAngleMotor.Set(ControlMode::PercentOutput,-0.2);

        // }else {
        //          frontRightAngleMotor.Set(ControlMode::PercentOutput,0);

        // }


    backRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backRightAngle, backRightAngleCalc)));

        // if (backRightAngle < backRightAngleCalc - 7.5) {
        //         backRightAngleMotor.Set(ControlMode::PercentOutput,0.2);

        // }else if (backRightAngle > backRightAngleCalc + 7.5){
        //         backRightAngleMotor.Set(ControlMode::PercentOutput,-0.2);

        // }else {
        //          backRightAngleMotor.Set(ControlMode::PercentOutput,0);

        // }

    backLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backLeftAngle, backLeftAngleCalc)));

        // if (backLeftAngle < backLeftAngleCalc - 7.5) {
        //         backLeftAngleMotor.Set(ControlMode::PercentOutput,0.2);

        // }else if (backLeftAngle > backLeftAngleCalc + 7.5){
        //         backLeftAngleMotor.Set(ControlMode::PercentOutput,-0.2);

        // }else {
        //          backLeftAngleMotor.Set(ControlMode::PercentOutput,0);

        // }
      OldbackRightSpeed =backRightSpeed;
      OldbackLeftSpeed = backLeftSpeed;
     OldfrontRightSpeed = frontRightSpeed;
     OldfrontLeftSpeed = frontLeftSpeed;
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