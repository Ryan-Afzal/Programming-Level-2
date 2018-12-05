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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class DriveTrain extends PIDSubsystem {

  private DifferentialDrive drive;
  private SpeedControllerGroup r;
  private SpeedControllerGroup l;

  private WPI_TalonSRX r0, r1, l0, l1;

  private ADXRS450_Gyro gyro;
  private PIDController pid;

  public static final double KP = 0.10;
  public static final double KI = 0.00;
  public static final double KD = 0.00;

  public DriveTrain() {
    super(KP, KI, KD);
    this.gyro = new ADXRS450_Gyro();
    this.gyro.calibrate();

    this.enable();

    this.r0 = new WPI_TalonSRX(RobotMap.rightDriveMotor1);
    this.r1 = new WPI_TalonSRX(RobotMap.rightDriveMotor2);
    this.l0 = new WPI_TalonSRX(RobotMap.leftDriveMotor1);
    this.l1 = new WPI_TalonSRX(RobotMap.leftDriveMotor2);

    this.r = new SpeedControllerGroup(r0, r1);
    this.l = new SpeedControllerGroup(l0, l1);

    r1.follow(r0);
    l1.follow(l0);

    this.r.setInverted(true);
    this.l.setInverted(true);

    this.drive = new DifferentialDrive(l, r);
  }

  public void turn(double setpoint) {
    this.setSetpoint(setpoint);
    this.enable();
  }

  @Override
  protected double returnPIDInput() {
    return this.getAngle();
  }

  public double getAngle() {
    return this.gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    System.out.println(output);//this.arcade(output, -output);
  }

  public void arcade(double moveValue, double rotateValue) {
    drive.arcadeDrive(moveValue, -rotateValue * 0.75);
  }

  public void arcadeNoConstants(double moveValue, double rotateValue) {
    drive.arcadeDrive(moveValue, rotateValue);
  }

  @Override
  public void initDefaultCommand() {
  }
}
