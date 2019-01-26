/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;

public class PanelHandler extends Subsystem {
  private TalonSRX hookMotor;
  private boolean hookMotorInverted = false;
  private static final double upSpeed = 1.0;
  private static final double downSpeed = -1.0;
  private static final double stopSpeed = 0.0;

  public enum Positions{
    UP, DOWN
  }
  public void setHookPosition(Positions position){
    switch(position){
      // Motor turns in about 0.625 seconds in a 90 degree rotation
      // Motor turns in about 2.50 seconds in a full (360 degrees) rotation
      case UP: 
      hookMotor.set(ControlMode.PercentOutput, upSpeed);
      break;
      
      case DOWN:
      hookMotor.set(ControlMode.PercentOutput, downSpeed);
      break;
    }
  }
  public void stop(){
    hookMotor.set(ControlMode.PercentOutput, stopSpeed);
  }

  public PanelHandler(){
    hookMotor = new TalonSRX(RobotMap.hookMotorID);
    
    hookMotor.setInverted(hookMotorInverted);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
