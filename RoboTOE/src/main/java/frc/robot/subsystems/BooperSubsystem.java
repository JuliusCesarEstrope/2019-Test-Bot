package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.BooperCommand;


public class BooperSubsystem extends Subsystem {
  static DoubleSolenoid booper;

  public BooperSubsystem(int[] booperPorts) {
    if (Constants.wristEnabled){
    booper = new DoubleSolenoid(booperPorts[0], booperPorts[1]);
    
    }
  }

  public void setBooperForward() {
    if (Constants.wristEnabled){
    booper.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void setBooperReverse() {
    if (Constants.wristEnabled){
    booper.set(DoubleSolenoid.Value.kReverse);
    }
  }
 
  public void initDefaultCommand() {
    if (Constants.wristEnabled){
    setDefaultCommand(new BooperCommand());
    }
  }
}