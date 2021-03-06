/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPIlib.h>

#include "OECPigeonIMU.h"
#include "Tankdrive.h"
#include "OECJoystick.h"

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
private:
  cs::UsbCamera* pointer2msLifeCam1;
  Tankdrive *tankdrive;

  SmartDashboard *dash;
  };
