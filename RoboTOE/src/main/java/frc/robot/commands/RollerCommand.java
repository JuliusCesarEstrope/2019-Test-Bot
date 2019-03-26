package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class RollerCommand extends CommandBase {
  
  boolean jogRollerHatchUp, jogRollerHatchDown, buttonReleased, shootRollerBall;
  Timer timer, ballTimer;
  double time = 0.05;
  double ballTime = 0.1;

  public RollerCommand() {
    requires(roller);
  }

  protected void initialize() {
    //RobotLog.putMessage("Running RollerCommand");
    roller.setRollerMotor(0);
    jogRollerHatchUp = false;
    jogRollerHatchDown = false;
    shootRollerBall = false;
    buttonReleased = true;
    timer = new Timer();
    ballTimer = new Timer();
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

    if(oi.getBallShootButton() && buttonReleased){
      shootRollerBall = true;
      ballTimer.start();
      buttonReleased = false;
    } else if(!oi.getBallShootButton()){
      buttonReleased = true;
    }

    if(timer.hasPeriodPassed(time)){
      timer.stop();
      timer.reset();
      jogRollerHatchUp = false;
      jogRollerHatchDown = false;
    }
    if(ballTimer.hasPeriodPassed(ballTime)){
      ballTimer.stop();
      ballTimer.reset();
      shootRollerBall = false;
    }

    if (oi.getRollerButtonIn()) {
      roller.setRollerMotor(-1.00);
    } else if (oi.getRollerButtonOut()) {
      roller.setRollerMotor(1.00);
    } else {
      roller.setRollerMotor(0);
    }
    if(jogRollerHatchDown){
      roller.setRollerMotor(-0.40);
    }
    else if(jogRollerHatchUp){
      roller.setRollerMotor(0.40);
    }
    else if(shootRollerBall){
      roller.setRollerMotor(-1.00);
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