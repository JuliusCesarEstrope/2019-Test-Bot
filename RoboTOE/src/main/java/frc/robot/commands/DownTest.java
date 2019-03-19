package frc.robot.commands;

public class DownTest extends CommandBase {
  public DownTest() {
    requires(wrist);
  }

  protected void initialize() {
    wrist.setBothWristMotor(-0.4);
  }

  protected void execute() {
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    wrist.setBothWristMotor(0);
  }

  protected void interrupted() {
    wrist.setBothWristMotor(0);
  }
}