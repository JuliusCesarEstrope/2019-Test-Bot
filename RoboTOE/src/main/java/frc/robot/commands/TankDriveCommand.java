package frc.robot.commands;



public class TankDriveCommand extends CommandBase {
  public TankDriveCommand() {
    requires(drive);
  }

  protected void initialize() {
    drive.setBoth(0, 0);
  }

  protected void execute() {
    drive.setBoth(oi.getleftYAxis(), oi.getrightYAxis());
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    drive.setBoth(0, 0);
  }

  protected void interrupted() {
    drive.setBoth(0, 0);
  }
}