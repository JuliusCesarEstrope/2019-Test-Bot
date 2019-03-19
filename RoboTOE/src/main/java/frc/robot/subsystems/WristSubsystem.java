package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class WristSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static VictorSPX rightWristMotor;
  static TalonSRX leftWristMotor;

  double tolerance = 70;

  public WristSubsystem(int leftWristMotorPort, int rightWristMotorPort, int[] leftWristEncoderPort,
  int[] rightWristEncoderPort, double[] wristPIDValues) {
    if (Constants.wristEnabled){

    leftWristMotor = new WPI_TalonSRX(leftWristMotorPort);
    rightWristMotor = new WPI_VictorSPX(rightWristMotorPort);

    leftWristMotor.configFactoryDefault();
    rightWristMotor.configFactoryDefault();

    rightWristMotor.follow(leftWristMotor); // :)
    leftWristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightWristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    leftWristMotor.setInverted(true); 
    rightWristMotor.setInverted(InvertType.OpposeMaster);

    leftWristMotor.configPeakOutputForward(0.4);
    leftWristMotor.configPeakOutputReverse(-0.4);
    leftWristMotor.configClosedLoopPeakOutput(0, 0.4);
    
    /*
    rightWristMotor.configPeakOutputForward(0.4);
    rightWristMotor.configPeakOutputReverse(-0.4);
    rightWristMotor.configClosedLoopPeakOutput(0, 0.4);
    */

    leftWristMotor.config_kP(0, wristPIDValues[0]);
    leftWristMotor.config_kI(0, wristPIDValues[1]);
    leftWristMotor.config_kD(0, wristPIDValues[2]);
    leftWristMotor.config_kF(0, wristPIDValues[3]);

    leftWristMotor.setNeutralMode(NeutralMode.Brake);
    rightWristMotor.setNeutralMode(NeutralMode.Brake);

    }
  }

  public void setLeftWristMotor(double Speed) {
    if (Constants.wristEnabled){
    leftWristMotor.set(ControlMode.PercentOutput, Speed);
    }
  }

  public void setRightWristMotor(double Speed) {
    if (Constants.wristEnabled){
    rightWristMotor.set(ControlMode.PercentOutput, Speed);
    }
  }

  public void ResetEncoder() {
    if (Constants.wristEnabled){
    leftWristMotor.setSelectedSensorPosition(0);
    rightWristMotor.setSelectedSensorPosition(0);
    }
  }

  public int getLeftWristEncoder() {
    if (Constants.wristEnabled){
    //return leftWristMotor.getSelectedSensorPosition();
    return leftWristMotor.getSelectedSensorPosition();
    // return leftWristEncoder.get();
    } else {
      return 0;
    }
  }

  public int getLeftWristEncoderError(){
    if (Constants.wristEnabled){
    return leftWristMotor.getClosedLoopError();
    } else {
      return 0;
    }
  }

  public int getRightWristEncoder() {
    if (Constants.wristEnabled){
    return rightWristMotor.getSelectedSensorPosition();
    // return rightWristEncoder.get();
    } else {
      return 0;
    }
  }

  public double getLeftTalonOutput() {
    if (Constants.wristEnabled){
    return leftWristMotor.getMotorOutputPercent();
    } else {
      return 0;
    }
  }

  public void setBothWristMotor(double pidWristOutput) {
    if (Constants.wristEnabled){
    setLeftWristMotor(pidWristOutput);
    setRightWristMotor(pidWristOutput);
    }
  }

  public void setBothWristMotor(double Speed, double Speed1) {
    if (Constants.wristEnabled){
    setLeftWristMotor(Speed * 0.4);
    setRightWristMotor(Speed * 0.4);
    }
  }

  public void setLeftWristMotorPosition(double position){
    if (Constants.wristEnabled){
    //rightWristMotor.set(ControlMode.Follower, );
    leftWristMotor.set(ControlMode.Position, position);
    }
  }

  public void setRightWristMotorPosition(double position){
    if (Constants.wristEnabled){
    rightWristMotor.set(ControlMode.Position, position);
    }
  }

  public boolean wristOnTarget(){
    return (Math.abs(leftWristMotor.getClosedLoopError()) < tolerance)? true:false;
  }

  public void initDefaultCommand() {
    if (Constants.wristEnabled){
  // setDefaultCommand(new ManualCommand());
    }
  }

}
