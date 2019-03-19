package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class DriveStraight extends CommandBase {
  public DriveStraight() {
    requires(drive);
  }

  protected void initialize() {
    SmartDashboard.putString("DB/Sring 9", "Running DriveStraight");
  }

  protected void execute() {
    drive.setBoth(oi.getrightYAxis(), oi.getrightYAxis());
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    drive.setBoth(0, 0);
    SmartDashboard.putString("DB/Sring 9", "Running DriveStraight");
  }

  protected void interrupted() {
    drive.setBoth(0, 0);
    SmartDashboard.putString("DB/String 9", "Interrupted DriveStraight");
  }
}