package frc.robot.commands;

public class RollerCommand extends CommandBase {
  public RollerCommand() {
    requires(roller);
  }

  protected void initialize() {
    //RobotLog.putMessage("Running RollerCommand");
    roller.setRollerMotor(0);
  }

  protected void execute() {
    if (oi.getRollerButtonIn()) {
      roller.setRollerMotor(-0.50);
    } else if (oi.getRollerButtonOut()) {
      roller.setRollerMotor(0.50);
    } else {
      roller.setRollerMotor(0);
    }
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
  }

  protected void interrupted() {
    roller.setRollerMotor(0);
  }
}