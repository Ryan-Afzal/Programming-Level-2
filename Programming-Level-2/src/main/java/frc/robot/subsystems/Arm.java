/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

public class Arm extends PIDSubsystem {
  private WPI_TalonSRX arm;
  private Servo flag;

  public static final double SPEED = 0.5;

  public static final double KP = 0.50;
  public static final double KI = 0.00;
  public static final double KD = 0.00;

  private int direction = 1;

  private double offset = 135. / 131;
  private double gearReduction = 30. / 54 * 1 / 2;
  public static final double startingAngle = -50;

  public Arm() {
    super("ArmPID", KP, KI, KD);
    this.setAbsoluteTolerance(0.2);
    this.getPIDController().setContinuous(false);

    this.arm = new WPI_TalonSRX(RobotMap.armMotor);
    this.arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    this.arm.setSensorPhase(false);
    this.arm.setSelectedSensorPosition(0, 0, 0);

    this.flag = new Servo(0);
    //servoStart = servoAngle();
  }

  @Override
  public void enable() {
    System.out.println("enabled!");
    super.enable();
  }

  @Override
  public void disable() {
    System.out.println("disabled!");
    super.disable();
  }

  public double getEncoderPosition() {
    return this.arm.getSelectedSensorPosition(0);
  }

  @Override
  protected double returnPIDInput() {
    return this.getArmAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.arm.pidWrite(output * direction);
  }

  public void startMotor(int direction) {
    this.direction = direction;
    this.enable();
  }

  public void stopMotor() {
    this.disable();
  }

  public double getArmAngle() {
    return Math.toRadians(this.getEncoderPosition() * gearReduction * offset / 4096 * 360 + startingAngle);
  }

  @Override
  public void initDefaultCommand() {
  }
}
