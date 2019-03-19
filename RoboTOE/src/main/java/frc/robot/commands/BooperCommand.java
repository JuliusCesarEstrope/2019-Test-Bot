package frc.robot.commands;

public class BooperCommand extends CommandBase {
  public BooperCommand() {
    requires(booper);
  }

  protected void initialize() {
    booper.setBooperReverse();
  }

  protected void execute() {
    if (oi.getBooperButton()) {
      booper.setBooperForward();
    } else {
      booper.setBooperReverse();
    }
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
  }

  protected void interrupted() {
  }
}