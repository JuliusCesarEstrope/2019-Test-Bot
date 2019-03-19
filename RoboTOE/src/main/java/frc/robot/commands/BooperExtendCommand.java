package frc.robot.commands;

public class BooperExtendCommand extends CommandBase {
  public BooperExtendCommand() {
    requires(booper);
  }

  protected void initialize() {
    booper.setBooperForward();
    
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