package frc.robot.commands;

public class ZeroEncodersCommand extends CommandBase {
  
  

  public ZeroEncodersCommand() {
  }

  protected void initialize() {
    drive.resetEncoder();
    wrist.ResetEncoder();
  }

  protected void execute() {
  }
  protected boolean isFinished() {
    return true;
  }

  protected void end() {
  }

  protected void interrupted() {
  }
}