package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class RollerCommand extends CommandBase {
  
  boolean jogRollerHatchUp, jogRollerHatchDown, buttonReleased;
  Timer timer;
  double time = 0.05;

  public RollerCommand() {
    requires(roller);
  }

  protected void initialize() {
    //RobotLog.putMessage("Running RollerCommand");
    roller.setRollerMotor(0);
    jogRollerHatchUp = false;
    jogRollerHatchDown = false;
    buttonReleased = true;
    timer = new Timer();
  }

  protected void execute() {

    if (oi.getJogRollerHatchUp() && buttonReleased) {
      jogRollerHatchUp = true;
      timer.start();
      buttonReleased = false;
    } else if (oi.getJogRollerHatchDown() && buttonReleased) {
      jogRollerHatchDown = true;
      timer.start();
      buttonReleased = false;
    } else if(!oi.getJogRollerHatchUp() && !oi.getJogRollerHatchDown()){
      buttonReleased = true;
    }

    if(timer.hasPeriodPassed(time)){
      timer.stop();
      timer.reset();
      jogRollerHatchUp = false;
      jogRollerHatchDown = false;
    }

    if (oi.getRollerButtonIn()) {
      roller.setRollerMotor(-0.50);
    } else if (oi.getRollerButtonOut()) {
      roller.setRollerMotor(0.70);
    } else {
      roller.setRollerMotor(0);
    }
    if(jogRollerHatchDown){
      roller.setRollerMotor(-0.40);
    }
    else if(jogRollerHatchUp){
      roller.setRollerMotor(0.40);
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