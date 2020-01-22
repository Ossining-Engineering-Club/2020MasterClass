#include "Tankdrive.h"
#include <iostream>

using namespace std;
bool drivePIDFirstRun = true;
OECPIDController *leftDrivePID;
OECPIDController *rightDrivePID;
double leftEncoderOffset = 0.0;
double rightEncoderOffset = 0.0;
Tankdrive::Tankdrive(int CanLeftFront, int CanLeftBack, int CanLeftTop, int CanRightFront, int CanRightBack, int CanRightTop)
    {

//change the ports from here    
    CanLeftFront = 01;
    CanLeftBack = 02;
    CanLeftTop = 03;
    CanRightFront = 04;
    CanRightBack = 05;
    CanRightTop = 06;
   
    AutonOverride = false;
    leftDrivePID = new OECPIDController(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, 1.0);
    rightDrivePID = new OECPIDController(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, 1.0);
    drivePIDFirstRun = true;
    
    
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
    IMUTalonSRX = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(2);
    
    //Declaration of Motor Ports for Drive Train
    LeftFrontDrive = new rev::CANSparkMax(CanLeftFront, rev::CANSparkMax::MotorType::kBrushless);
    LeftBackDrive = new rev::CANSparkMax(CanLeftBack, rev::CANSparkMax::MotorType::kBrushless);
    LeftTopDrive = new rev::CANSparkMax(CanLeftTop, rev::CANSparkMax::MotorType::kBrushless);
    RightFrontDrive = new rev::CANSparkMax(CanRightFront, rev::CANSparkMax::MotorType::kBrushless);
    RightBackDrive = new rev::CANSparkMax(CanRightBack, rev::CANSparkMax::MotorType::kBrushless);
    RightTopDrive = new rev::CANSparkMax(CanRightTop, rev::CANSparkMax::MotorType::kBrushless);

//PigeonIMU declarations    
    pigeonIMU = new OECPigeonIMU(01); //CHANGE THIS BACK!!!
    pigeonIMU->ResetAngle();
    
//ask if he only uses the back encoder or you take the average of all three encoders
    rightNeoEncoder = new rev::CANEncoder(*RightBackDrive);
    leftNeoEncoder = new rev::CANEncoder(*LeftBackDrive);
    
//resetting the encoders to 0
    leftEncoder->Reset();
    rightEncoder->Reset();

//throttle count
    throttle = 1.0;
    
    myTimer = new Timer();
    loopMode = internal;
}

void Tankdrive::ResetEncoders(){
    leftEncoderOffset = GetLeftEncoderDist();
    rightEncoderOffset = GetRightEncoderDist();
}
