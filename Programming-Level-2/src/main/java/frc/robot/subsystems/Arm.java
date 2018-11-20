/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Arm extends Subsystem {

  public WPI_TalonSRX arm, flag;

  public Arm() {
    this.arm = new WPI_TalonSRX(RobotMap.armMotor);
    this.flag = new WPI_TalonSRX(RobotMap.hookLockServo);
  }
  
  public void setArmSpeed(int direction) {

  }

  public void setFlagSpeed(int direction) {

  }

  public double getArmAngle() {
    return 0.0;//this.arm.getSelectedSensorPosition() / 2;
  }

  @Override
  public void initDefaultCommand() {
  }
}
