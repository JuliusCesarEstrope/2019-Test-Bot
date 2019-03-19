package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristCommand extends CommandBase {

  int leftWristEncoder;
  int rightWristEncoder;
  double wristSetPoint;
  Timer timer;

  public WristCommand(double wristSetPoint) {
    requires(wrist);
    // Use requires() here to declare subsystem dependencies
    this.wristSetPoint = wristSetPoint;
    timer = new Timer();

  }

  protected void initialize() {
    timer.start();
    // Set point
    // Send PID to correct motors
    SmartDashboard.putString("DB/String 7", "Running WristCommand");
  }

  @Override
  protected void execute() {
    // Two Positions (Up, Down)
    //wrist.setRightWristMotorPosition(wristSetPoint);
    wrist.setLeftWristMotorPosition(wristSetPoint);
    wrist.setRightWristMotor(wrist.getLeftTalonOutput());
  }

  protected boolean isFinished() {
    // if needed to maintain set pont (false) stopping point (true)
    if (!wrist.wristOnTarget()) {
      timer.reset();
    } else {
      if (timer.get() > 0.5) {
        return true;

      }
    }
    return false;
  }
  

  protected void end() {
   // drive.setBoth(0, 0);
    wrist.setBothWristMotor(0, 0);
    SmartDashboard.putString("DB/String 7", "Ended WristCommand");
  }

  protected void interrupted() {
    wrist.setBothWristMotor(0, 0);
    SmartDashboard.putString("DB/String 7", "Interrupted WristCommand");
  }
}