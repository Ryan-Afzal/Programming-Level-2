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

public class Arm extends PIDSubsystem {
  private WPI_TalonSRX arm;
  private Servo flag;

  public static final double SPEED = 0.5;

  private int direction = 1;

  private double offset = 135. / 131;
  private double gearReduction = 30. / 54 * 1 / 2;
  public static final double startingAngle = -50;

  public Arm() {
    super("Arm", 1.0, 0.0, 0.0);
    this.setAbsoluteTolerance(0.2);
    this.getPIDController().setContinuous(false);

    this.arm = new WPI_TalonSRX(RobotMap.armMotor);
    
    this.flag = new Servo(0);
    //servoStart = servoAngle();
  }

  public double getEncoderPosition() {
    return this.arm.getSelectedSensorPosition(0);
  }

  @Override
  protected double returnPIDInput() {
    return this.arm.getSelectedSensorPosition(0);
  }

  @Override
  protected void usePIDOutput(double output) {
    this.arm.pidWrite(direction * output);
  }

  public void startMotor(int direction) {
    this.direction = direction;
    this.enable();
  }

  public void stopMotor() {
    this.disable();
  }

  public double getArmAngle() {
    return Math.toRadians(getEncoderPosition() * gearReduction * offset / 4096 * 360 + startingAngle);
  }

  @Override
  public void initDefaultCommand() {
  }
}
