package frc.robot.commands;

public class ZeroButtonCommand extends CommandBase {
  public ZeroButtonCommand() {
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