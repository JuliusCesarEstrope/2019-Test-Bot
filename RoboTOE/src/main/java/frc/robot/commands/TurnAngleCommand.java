package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnAngleCommand extends CommandBase {
  public double angle;
  Timer timer;
  boolean endCommand;

  public TurnAngleCommand(double angle) {
    requires(drive);
    this.angle = angle;
    timer = new Timer();
  }

  protected void initialize() {
    SmartDashboard.putString("DB/String 6", "Running Turn Angle");
    drive.resetGyro();
    drive.setGyroSetpoint(angle);
    timer.start(); 
  }

  protected void execute() {
    SmartDashboard.putString("DB/String 5", "" + timer.get());
    drive.setBoth(drive.getGyroPIDOutput(), -drive.getGyroPIDOutput());
  }

  protected boolean isFinished() {
    if (!drive.gyroPIDOnSetpoint()) {
      timer.reset();
    } else {
      return timer.hasPeriodPassed(0.5);
    }
    return false;
  }

  protected void end() {
    SmartDashboard.putString("DB/String 6", "Ended Turn Angle");
    endCommand = true;
    drive.setBoth(0, 0);
  }

  protected void interrupted() {
    SmartDashboard.putString("DB/String 6", "Interrupted Turn Angle");
    drive.setBoth(0, 0);
  }
}