/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class DriveTrain extends Subsystem {

  private DifferentialDrive drive;
  private SpeedControllerGroup r;
  private SpeedControllerGroup l;

  public WPI_TalonSRX r0, r1, l0, l1;

  public DriveTrain() {
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
  public void arcade(double moveValue, double rotateValue) {
    drive.arcadeDrive(moveValue, -rotateValue);
  }

  public void arcadeNoConstants(double moveValue, double rotateValue) {
    drive.arcadeDrive(moveValue, rotateValue);
  }

  @Override
  public void initDefaultCommand() {
  }
}
