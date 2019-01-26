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

  public DriveTrain(){
  leftMotorMaster = new TalonSRX(RobotMap.leftMotorMaster);
  leftMotorFollower = new TalonSRX(RobotMap.leftMotorFollower);
  rightMotorMaster = new TalonSRX(RobotMap.rightMotorMaster);
  rightMotorFollower = new TalonSRX(RobotMap.rightMotorFollower);

  reverseOutputRight = true;
  reverseOutputLeft = false;

  rightMotorMaster.setInverted(reverseOutputRight);
  rightMotorFollower.setInverted(reverseOutputRight);

  leftMotorMaster.setInverted(reverseOutputLeft);
  leftMotorFollower.setInverted(reverseOutputLeft);

  leftMotorFollower.set(ControlMode.Follower,RobotMap.leftMotorMaster);
  rightMotorFollower.set(ControlMode.Follower,RobotMap.rightMotorMaster);
  
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
