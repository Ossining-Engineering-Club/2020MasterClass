#pragma once
#include <frc/WPIlib.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "OECPigeonIMU.h"
#include "OECJoystick.h"
#include "OECPIDController.h"
#include <ctre/Phoenix.h>

using namespace frc;
class Tankdrive{
    public:
        bool AutonOverride;
        OECJoystick *stickLeft;
        OECJoystick *stickRight;
        OECJoystick *stickUtil;
        enum LoopMode{internal = 0, external = 1};
        enum DriveSide{left = 0, right = 1};

        Tankdrive(int CanLeftFront, int CanLeftBack, int CanLeftTop, int CanRightFront, int CanRightBack, int CanRightTop);
        OECPigeonIMU* GetPigeonIMU();
        void DriveStraightGyro(double power, double distInches, double startupTime, bool stopAtEnd);
        void TurnToHeading(double maxPower, double headingDegrees, float TimeOut);
        void SetPower(double leftPower, double rightPower);
        void SetThrottle(double throttle);
        void ResetEncoders();
        void SetLoopMode(LoopMode mode);
        double GetRightEncoderDist();
        int GetRightEncoderRaw();
        double GetLeftEncoderDist();
        int GetLeftEncoderRaw();
        void PIDSetPower(double leftPower, double rightPower);
        void TurnToTarget(double power);
        void DriveCurveEncoder(double radius, double degrees, double avgPower, double startupTime, bool stopAtEnd);
        void DriveGyro(double degreesPerInch, double degrees, double avgPower, double timeoutSec);
        void DriveGyroByRadius(double radius, double degrees, double avgPower, double timeoutSec);
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX* GetTalonSRX();
    private:
        frc::Timer AutoTimer;
        SmartDashboard *dash;
        OECPIDController *pidController;
        OECPigeonIMU *pigeonIMU;
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX *IMUTalonSRX;
        rev::CANSparkMax *LeftFrontDrive;
        rev::CANSparkMax *LeftBackDrive;
        rev::CANSparkMax *LeftTopDrive;
        rev::CANSparkMax *RightFrontDrive;
        rev::CANSparkMax *RightBackDrive;
        rev::CANSparkMax *RightTopDrive;
        rev::CANEncoder *leftNeoEncoder;
        rev::CANEncoder *rightNeoEncoder;
        frc::Encoder *leftEncoder;
        frc::Encoder *rightEncoder;
        double throttle;
        Timer *myTimer;
        LoopMode loopMode;
            
};