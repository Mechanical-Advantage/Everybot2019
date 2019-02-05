/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TunableNumber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VelocityPIDTuner extends Command {
  private TunableNumber setPoint = new TunableNumber("PIDTuner/setPoint");
  private TunableNumber p = new TunableNumber("PIDTuner/P");
  private TunableNumber i = new TunableNumber("PIDTuner/I");
  private TunableNumber d = new TunableNumber("PIDTuner/D");
  private TunableNumber f = new TunableNumber("PIDTuner/F");

  public VelocityPIDTuner() {
    requires(Robot.driveTrain_subsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    p.setDefault(Robot.driveTrain_subsystem.getP());
    i.setDefault(Robot.driveTrain_subsystem.getI());
    d.setDefault(Robot.driveTrain_subsystem.getD());
    f.setDefault(Robot.driveTrain_subsystem.getF());
    setPoint.setDefault(0);
    SmartDashboard.putBoolean("PIDTuner/enabled", false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentVelocity;
    Robot.driveTrain_subsystem.setPID(0, p.get(), i.get(), d.get(), f.get(), 0);
    if (SmartDashboard.getBoolean("PIDTuner/enabled", false) == true) {
      Robot.driveTrain_subsystem.driveInchesPerSecond(setPoint.get(), setPoint.get());
      currentVelocity = ((Robot.driveTrain_subsystem.getVelocityLeft() + Robot.driveTrain_subsystem.getVelocityRight())/2);
      SmartDashboard.putNumber("PIDTuner/currentVelocity", currentVelocity);
    } else {
      Robot.driveTrain_subsystem.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
