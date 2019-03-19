package frc.robot.commands;

public class BooperRetractCommand extends CommandBase {
  public BooperRetractCommand() {
    requires(booper);
  }

  protected void initialize() {
    booper.setBooperReverse();
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