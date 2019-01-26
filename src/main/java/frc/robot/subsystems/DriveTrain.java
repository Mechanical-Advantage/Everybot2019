/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private boolean reverseOutputLeft;
  private boolean reverseOutputRight;

  private TalonSRX leftMotorMaster;
  private TalonSRX leftMotorFollower;
  private TalonSRX rightMotorFollower;
  private TalonSRX rightMotorMaster;
  private static final double stopSpeed = 0.0;

  public DriveTrain() {
    leftMotorMaster = new TalonSRX(RobotMap.leftMotorMasterID);
    leftMotorFollower = new TalonSRX(RobotMap.leftMotorFollowerID);
    rightMotorMaster = new TalonSRX(RobotMap.rightMotorMasterID);
    rightMotorFollower = new TalonSRX(RobotMap.rightMotorFollowerID);

    reverseOutputRight = true;
    reverseOutputLeft = false;

    rightMotorMaster.setInverted(reverseOutputRight);
    rightMotorFollower.setInverted(reverseOutputRight);

    leftMotorMaster.setInverted(reverseOutputLeft);
    leftMotorFollower.setInverted(reverseOutputLeft);

    leftMotorFollower.set(ControlMode.Follower, RobotMap.leftMotorMasterID);
    rightMotorFollower.set(ControlMode.Follower, RobotMap.rightMotorMasterID);

  }

  public void drive(double leftPercent, double rightPercent) {
    leftMotorMaster.set(ControlMode.PercentOutput, leftPercent);
    rightMotorMaster.set(ControlMode.PercentOutput, rightPercent);
  }

  public void stop() {
    leftMotorMaster.set(ControlMode.PercentOutput, stopSpeed);
    rightMotorMaster.set(ControlMode.PercentOutput, stopSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
