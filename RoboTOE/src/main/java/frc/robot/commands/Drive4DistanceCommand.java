package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive4DistanceCommand extends CommandBase {

  double distance;
  // Timer timer;

  public Drive4DistanceCommand(double distance) {
    requires(drive);
    this.distance = distance;
    // timer = new Timer();
  }

  protected void initialize() {
    // RobotLog.putMessage("Running DynamicBrakingCommand");
    drive.resetEncoder();
    // drive.resetGyro();
    // timer.start();
  }

  protected void execute() {
    drive.setBothPositions(distance, distance/* , drive.getGyroPIDOutput() */);
    SmartDashboard.putString("DB/String 0", "Right Readout");
    SmartDashboard.putString("DB/String 5", "" + drive.getRightSensorPosition());
    SmartDashboard.putString("DB/String 1", "Right Error");
    SmartDashboard.putString("DB/String 6", "" + drive.getRightError());
  }

  protected boolean isFinished() {
    // return drive.distanceOnTarget() || ( (timer.get())>4);
    return false;
  }

  protected void end() {
    drive.setBoth(0);
  }

  protected void interrupted() {
    drive.setBoth(0);
  }
}