/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.*;
import frc.robot.*;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX w;

  public Wrist() {
    this.w = new WPI_TalonSRX(RobotMap.wristMotor);
  }

  public void setSpeed(double speed) {
    this.w.set(speed);
  }

  public void stop() {
    this.w.set(0);
  }

  @Override
  public void initDefaultCommand() {
  }
}
