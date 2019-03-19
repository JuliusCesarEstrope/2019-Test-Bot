package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.RollerCommand;

public class RollerSubsystem extends Subsystem {
  static VictorSPX rollerMotor;

  public RollerSubsystem(int rollerMotorPort) {
    if (Constants.wristEnabled){
    rollerMotor = new WPI_VictorSPX(rollerMotorPort);
    rollerMotor.setInverted(false);
    //RobotLog.putMessage("Running RollerSubsystem");
    }
  }

  public void setRollerMotor(double Speed) {
    if (Constants.wristEnabled){
    rollerMotor.set(ControlMode.PercentOutput, Speed);
    }
  }

  public void initDefaultCommand() {
    if (Constants.wristEnabled){
    setDefaultCommand(new RollerCommand());
    }
  }
}