// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <iostream>
#include "ctre/Phoenix.h"
#include "Robot.h"
#include "rev/CANSparkMax.h"

TalonFX fxL1(11);
TalonFX fxL2(12);
TalonFX fxR1(13);
TalonFX fxR2(14);
TalonSRX Lift(1);
rev::CANSparkMax m_sL{19, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_sR{18, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_intake{20, rev::CANSparkMax::MotorType::kBrushless};
 

frc::XboxController xboxx{0};
frc::Timer mt;

void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
  mt.Start();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  mt.Start();
  mt.Reset();
}
void Robot::AutonomousPeriodic() {
  /*
      while(mt.Get()<0.2){}
      fxL1.Set(ControlMode::PercentOutput, 0.35);
      fxL2.Set(ControlMode::PercentOutput, 0.35);
      fxR1.Set(ControlMode::PercentOutput, -0.35);
      fxR2.Set(ControlMode::PercentOutput, -0.35);
      while(mt.Get()<2.7){}
      fxL1.Set(ControlMode::PercentOutput, 0);
      fxL2.Set(ControlMode::PercentOutput, 0);  
      fxR1.Set(ControlMode::PercentOutput, 0);
      fxR2.Set(ControlMode::PercentOutput, 0);
      while(mt.Get()<3.2){}
      fxT1.Set(ControlMode::PercentOutput,-0.7);  
      while(mt.Get()<4.5){}
      fxT1.Set(ControlMode::PercentOutput,0);  
      while(mt.Get()<15.1){}
  */
    /*
      while((double)mt.Get()<0.2){}
      fxL1.Set(ControlMode::PercentOutput, -0.35);
      fxL2.Set(ControlMode::PercentOutput, -0.35);
      fxR1.Set(ControlMode::PercentOutput, 0.35);
      fxR2.Set(ControlMode::PercentOutput, 0.35);
      while((double)mt.Get()<3){}
      fxL1.Set(ControlMode::PercentOutput, 0);
      fxL2.Set(ControlMode::PercentOutput, 0);
      fxR1.Set(ControlMode::PercentOutput, 0);
      fxR2.Set(ControlMode::PercentOutput, 0);
      while((double)mt.Get()<15){}
    */
}

void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {
  /*
      Lt=xboxx.GetLeftTriggerAxis();
      Rt=xboxx.GetRightTriggerAxis();
      if(Rt==0 && Lt==0){
        fxT1.Set(ControlMode::PercentOutput,0);
      }
      else{
        fxT1.Set(ControlMode::PercentOutput,Rt>0?-0.7:(Lt>0?1:0));
      }
    fxL1.Set(ControlMode::PercentOutput, -Tnka*xboxx.GetLeftY());
    fxL2.Set(ControlMode::PercentOutput, -Tnka*xboxx.GetLeftY());
    fxR1.Set(ControlMode::PercentOutput, Tnka*xboxx.GetRightY());
    fxR2.Set(ControlMode::PercentOutput, Tnka*xboxx.GetRightY());
    */
      
      ax=-xboxx.GetLeftY();
      ay=xboxx.GetRightX();
      if((ax>0?ax:-ax)<0.1)ax=0;
      if((ay>0?ay:-ay)<0.1)ay=0; 
      Lp=Tnka*(ay+ax>1?1:(ay+ax<-1?-1:ay+ax));
      Rp=Tnka*(ay-ax>1?1:(ay-ax<-1?-1:ay-ax));
      if(xboxx.GetRawButton(5)){
        lft=1;
      }
      else if(xboxx.GetRawButton(6)){
        lft=-1;
      }
      else{
        lft=0;
      }
      Lt=xboxx.GetLeftTriggerAxis();
      Rt=xboxx.GetRightTriggerAxis();
      Lift.Set(ControlMode::PercentOutput,Lt*1);
      m_sL.Set(-wnka*Rt);
      m_sR.Set(wnka*Rt);
      //m_intake.Set(0.5*Lt);

      if(xboxx.GetRawButton(1)){
        Tnka=0.1;
      }
      if(xboxx.GetRawButton(2)){
        Tnka=0.4;
      }
      if(xboxx.GetRawButton(3)){
        Tnka=0.25;
      }
      if(xboxx.GetRawButton(4)){
        Tnka=1;
      }

      
      fxL1.Set(ControlMode::PercentOutput, Lp);
      fxL2.Set(ControlMode::PercentOutput, Lp);
      fxR1.Set(ControlMode::PercentOutput, Rp);
      fxR2.Set(ControlMode::PercentOutput, Rp);
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
