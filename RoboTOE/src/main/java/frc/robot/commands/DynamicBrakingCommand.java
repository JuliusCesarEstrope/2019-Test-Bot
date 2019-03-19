package frc.robot.commands;



public class DynamicBrakingCommand extends CommandBase {
  
  public DynamicBrakingCommand() {
    requires(drive);
  }

  protected void initialize() {
   // RobotLog.putMessage("Running DynamicBrakingCommand");
    drive.resetEncoder();
  }

  protected void execute() {
    drive.setBothPositions(0, 0);
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    drive.setBoth(0);
  }

  protected void interrupted() {
    drive.setBoth(0, 0);
  }
}