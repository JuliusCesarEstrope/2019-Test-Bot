package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class WaitCommand extends CommandBase {
  
  Timer timer;
  double time;
  
  public WaitCommand(double time) {
		timer = new Timer();
    this.time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.start();
    
		drive.setBoth(0);
    
		//RobotLog.putMessage("Running Drive4Distance: " + distanceToTravel);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.hasPeriodPassed(time);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

		//RobotLog.putMessage("Finished Drive4Distance: " + distanceToTravel);
		//RobotLog.putMessage("Distance traveled: " + leftEndDistance + ", " + rightEndDistance);
		drive.setBoth(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //RobotLog.putMessage("Interrupted Drive4Distance: " + distanceToTravel);
		drive.setBoth(0);
  }
}
