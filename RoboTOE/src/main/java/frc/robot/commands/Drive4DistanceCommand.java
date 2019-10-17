package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive4DistanceCommand extends CommandBase {

  double distance;
  Timer timer;

  public Drive4DistanceCommand(double distance) {
    requires(drive);
    this.distance = distance;
    drive.resetEncoder();
    drive.resetGyro();
    timer = new Timer();
  }

  protected void initialize() {
    // RobotLog.putMessage("Running DynamicBrakingCommand");
    SmartDashboard.putString("DB/String 9", "Running Drive4Distance");
    drive.resetEncoder();
    drive.resetGyro();
    timer.start();
    timer.reset();
  }

  protected void execute() {
    drive.setBothPositions(distance, distance/* , drive.getGyroPIDOutput() */);
    SmartDashboard.putString("DB/String 0", "Left Readout");
    SmartDashboard.putString("DB/String 5", "" + drive.getWheelDistanceLeft());
    SmartDashboard.putString("DB/String 1", "Left Error");
    SmartDashboard.putString("DB/String 6", "" + drive.getLeftError());
  }

  protected boolean isFinished() {
    if (!drive.distanceOnTarget()) {
      timer.reset();
    } else {
      return timer.hasPeriodPassed(0.5);
    }
    return false;
    //return false;
  }

  protected void end() {
    drive.setBoth(0);
    SmartDashboard.putString("DB/String 9", "Ended Drive4Distance");
  }

  protected void interrupted() {
    drive.setBoth(0);
    SmartDashboard.putString("DB/String 9", "Interrupted Drive4Distance");
  }
}