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
public class BallHandler extends Subsystem {
  private TalonSRX ballHandlerMotor;
  private boolean ballHandlerMotorInverted = false;
  private static final double stopSpeed = 0.0;

  public BallHandler() {
    ballHandlerMotor = new TalonSRX(RobotMap.ballHandlerMotorID);

    ballHandlerMotor.setInverted(ballHandlerMotorInverted);
  }

  public void lift() {

  }

  public void stop() {
    ballHandlerMotor.set(ControlMode.PercentOutput, stopSpeed);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
