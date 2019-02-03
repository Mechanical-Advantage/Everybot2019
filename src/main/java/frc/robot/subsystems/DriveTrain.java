/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private boolean reverseOutputLeft;
  private boolean reverseOutputRight;

  private boolean leftSetSensorReversed;
  private boolean rightSetSensorReversed;

  private TalonSRX leftMotorMaster;
  private TalonSRX leftMotorFollower;
  private TalonSRX rightMotorFollower;
  private TalonSRX rightMotorMaster;

  private static final int maxVelocity = 180; // guess in inches per second

  private static final int ticksPerRotation = 1440;

  private static final int kIzone = 0;

  private static final double wheelDiameter = 6; // inches

  private static final double kP = -1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 1;

  private static final double stopSpeed = 0.0;

  public DriveTrain() {
    leftMotorMaster = new TalonSRX(RobotMap.leftMotorMasterID);
    leftMotorFollower = new TalonSRX(RobotMap.leftMotorFollowerID);

    rightMotorMaster = new TalonSRX(RobotMap.rightMotorMasterID);
    rightMotorFollower = new TalonSRX(RobotMap.rightMotorFollowerID);

    reverseOutputRight = false;
    reverseOutputLeft = true;

    leftSetSensorReversed = false;
    rightSetSensorReversed = false;

    leftMotorMaster.setInverted(reverseOutputLeft);
    leftMotorFollower.setInverted(reverseOutputLeft);

    rightMotorMaster.setInverted(reverseOutputRight);
    rightMotorFollower.setInverted(reverseOutputRight);

    leftMotorFollower.set(ControlMode.Follower, RobotMap.leftMotorMasterID);
    rightMotorFollower.set(ControlMode.Follower, RobotMap.rightMotorMasterID);

    leftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    leftMotorMaster.setSensorPhase(leftSetSensorReversed);
    rightMotorMaster.setSensorPhase(rightSetSensorReversed);

    leftMotorMaster.configNominalOutputForward(0);
    leftMotorMaster.configNominalOutputReverse(0);
    rightMotorMaster.configNominalOutputForward(0);
    rightMotorMaster.configNominalOutputReverse(0);

    leftMotorMaster.configPeakOutputForward(1);
    leftMotorMaster.configPeakOutputReverse(-1);
    rightMotorMaster.configPeakOutputForward(1);
    rightMotorMaster.configPeakOutputReverse(-1);

    setPID(0, kP, kI, kD, kF, kIzone);
  }

  public void setPID(int slotIdx, double PValue, double IValue, double DValue, double FValue, int izone) {
    leftMotorMaster.config_kP(slotIdx, PValue);
    leftMotorMaster.config_kI(slotIdx, IValue);
    leftMotorMaster.config_kD(slotIdx, DValue);
    leftMotorMaster.config_kF(slotIdx, FValue);
    leftMotorMaster.config_IntegralZone(slotIdx, izone);

    rightMotorMaster.config_kP(slotIdx, PValue);
    rightMotorMaster.config_kI(slotIdx, IValue);
    rightMotorMaster.config_kD(slotIdx, DValue);
    rightMotorMaster.config_kF(slotIdx, FValue);
    rightMotorMaster.config_IntegralZone(slotIdx, izone);
  }

  public double getVelocityLeft() {
    return (leftMotorMaster.getSelectedSensorVelocity() * wheelDiameter * Math.PI * 10) / (ticksPerRotation);
  }

  public double getVelocityRight() {
    return (rightMotorMaster.getSelectedSensorVelocity() * wheelDiameter * Math.PI * 10) / (ticksPerRotation);
  }

  public void driveInchesPerSecond(double leftIPS, double rightIPS) {
    drive((((leftIPS * ticksPerRotation) / (wheelDiameter * Math.PI * 10)) / maxVelocity), // still need to fix units.
        (((rightIPS * ticksPerRotation) / (wheelDiameter * Math.PI * 10)) / maxVelocity));
  }

  public void drive(double leftPercent, double rightPercent) {
    if (Robot.oi.getDriveDisabled()) {
      stop();
    } else {
      if (Robot.oi.getDriveOpenLoop()) {
        leftMotorMaster.set(ControlMode.PercentOutput, leftPercent);
        rightMotorMaster.set(ControlMode.PercentOutput, rightPercent);
      } else {
        // driveclosedloop code
        leftPercent *= RobotMap.maxVelocity;
        rightPercent *= RobotMap.maxVelocity;

        leftMotorMaster.set(ControlMode.Velocity, leftPercent);
        rightMotorMaster.set(ControlMode.Velocity, rightPercent);
      }
    }
  }

  public void stop() {
    leftMotorMaster.set(ControlMode.PercentOutput, stopSpeed);
    rightMotorMaster.set(ControlMode.PercentOutput, stopSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new JoystickDrive());
  }
}
