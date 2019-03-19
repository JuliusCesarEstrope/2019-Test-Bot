package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.TankDriveCommand;
import frc.robot.Utilities.DoublePIDOutput;

public class DriveSubsystem extends Subsystem {
  TalonSRX talonLeft, talonRight;
  DoublePIDOutput gyroOutput;
  VictorSPX[] victorsLeft, victorsRight;
  AnalogGyro gyroDrive;
  double pidTolerance = 70;
  double ramp = .25;
  static DigitalInput leftSensor;
  static DigitalInput frontSensor;
  static DigitalInput rightSensor;
  static DigitalInput backSensor;
  private static PIDController gyroPID;
  private static int tolerance = 70;

  public DriveSubsystem(int[] motorPortsLeft, int[] motorPortsRight, int gyroPort, int encoderPortLeft[],
      int encoderPortRight[], int frontSensor, int rightSensor, int backSensor, int leftSensor,
      int[] driveEncoderPortLeft, int[] driveEncoderPortRight, double circumferanceOfWheels, double ticksOfEncoder,
      double[] drivePIDValues) {
    if (Constants.wristEnabled) {

      gyroDrive = new AnalogGyro(gyroPort);
      talonLeft = new WPI_TalonSRX(motorPortsLeft[0]);
      talonRight = new WPI_TalonSRX(motorPortsRight[0]);
      gyroOutput = new DoublePIDOutput();

      talonLeft.configFactoryDefault();
      talonRight.configFactoryDefault();
      talonLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
      talonRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

      talonLeft.configPeakOutputForward(1);
      talonLeft.configPeakOutputReverse(-1);
      talonLeft.configClosedloopRamp(ramp);
      // talonLeft.config_kP(0, drivePIDValues[0]);
      // talonLeft.config_kI(0, drivePIDValues[1]);
      // talonLeft.config_kD(0, drivePIDValues[2]);
      // talonLeft.config_kF(0, drivePIDValues[3]);
      // talonLeft.setSensorPhase(true);
      // talonLeft.setInverted(false);

      talonRight.configPeakOutputForward(1);
      talonRight.configPeakOutputReverse(-1);
      talonRight.configClosedloopRamp(ramp);
      // talonRight.config_kP(0, drivePIDValues[0]);
      // talonRight.config_kI(0, drivePIDValues[1]);
      // talonRight.config_kD(0, drivePIDValues[2]);
      // talonRight.config_kF(0, drivePIDValues[3]);
      // talonRight.setSensorPhase(true);
      talonRight.setInverted(true);

      talonLeft.setNeutralMode(NeutralMode.Brake);
      talonRight.setNeutralMode(NeutralMode.Brake);

      gyroPID = new PIDController(0.05, 0, 0, gyroDrive, gyroOutput);
      gyroPID.setAbsoluteTolerance(1);
      gyroPID.setInputRange(-Integer.MAX_VALUE, Integer.MAX_VALUE);
      gyroPID.setOutputRange(-1, 1);
      gyroPID.setEnabled(true);

      victorsLeft = new WPI_VictorSPX[motorPortsLeft.length - 1];
      victorsRight = new WPI_VictorSPX[motorPortsRight.length - 1];

      for (int i = 0; i < victorsLeft.length; i++)
        victorsLeft[i] = new WPI_VictorSPX(motorPortsLeft[i + 1]);
      for (int i = 0; i < victorsRight.length; i++)
        victorsRight[i] = new WPI_VictorSPX(motorPortsRight[i + 1]);

      for (int i = 0; i < victorsLeft.length; i++) {
        victorsLeft[i].follow(talonLeft);
        victorsLeft[i].setNeutralMode(NeutralMode.Brake);
        victorsLeft[i].setInverted(InvertType.FollowMaster);
      }

      for (int i = 0; i < victorsRight.length; i++) {
        victorsRight[i].follow(talonRight);
        victorsRight[i].setNeutralMode(NeutralMode.Brake);
        victorsRight[i].setInverted(InvertType.FollowMaster);
      }

    }

  }

  public void resetGyro() {
    if (Constants.wristEnabled) {
      gyroDrive.reset();
    }
  }

  public void setgyroPIDValues(double p, double i, double d) {
    if (Constants.wristEnabled) {
      gyroPID.setPID(p, i, d);
    }
  }

  public void setBothWheelEncoders(double p, double i, double d) {
    if (Constants.wristEnabled) {
      // talonLeft.setPID (p, i, d);
      talonLeft.config_kP(0, p);
      talonLeft.config_kI(0, i);
      talonLeft.config_kD(0, d);

      // talonRight.setPID (p, i, d);
      talonRight.config_kP(0, p);
      talonRight.config_kI(0, i);
      talonRight.config_kD(0, d);
    }
  }

  /*
   * public void setBothWheelEncoders(double p, double i, double d) {
   * leftWheelEncoderPID.setPID (p, i, d); leftWheelEncoderPID.setP(p);
   * leftWheelEncoderPID.setI(i); leftWheelEncoderPID.setD(d);
   * leftWheelEncoderPID.setF(f);
   * 
   * rightWheelEncoderPID.setPID (p, i, d); rightWheelEncoderPID.setP(p);
   * rightWheelEncoderPID.setI(i); rightWheelEncoderPID.setD(d);
   * rightWheelEncoderPID.setF(f); }
   */

  public void enableGyroPID() {
    if (Constants.wristEnabled) {
      gyroPID.enable();
    }
  }

  public double getGyroPIDError() {
    if (Constants.wristEnabled) {
      return gyroPID.getError();
    } else {
      return 0;
    }
  }

  public void setGyroSetpoint(double setpoint) {
    if (Constants.wristEnabled) {
      gyroPID.setSetpoint(setpoint);
    }
  }

  public boolean gyroPIDOnSetpoint() {
    if (Constants.wristEnabled) {
      return gyroPID.onTarget();
    } else {
      return false;
    }
  }

  public boolean rightOnTarget() {
    if (Constants.wristEnabled) {
      return Math.abs(talonRight.getClosedLoopError()) < tolerance;
    } else {
      return false;
    }
  }

  public boolean leftOnTarget() {
    if (Constants.wristEnabled) {
      return Math.abs(talonLeft.getClosedLoopError()) < tolerance;
    } else {
      return false;
    }
  }

  public boolean distanceOnTarget() {
    if (Constants.wristEnabled) {
      return leftOnTarget() || rightOnTarget();
    } else {
      return false;
    }
  }

  public double getGyroPIDOutput() {
    if (Constants.wristEnabled) {
      return gyroPID.get();
    } else {
      return 0;
    }
  }

  public void setBothWheelPIDValues(double p, double i, double d) {
    if (Constants.wristEnabled) {
      // leftWheelEncoderPID.setPID (p, i, d);
      // leftWheelEncoderPID.setP(p);
      // leftWheelEncoderPID.setI(i);
      // leftWheelEncoderPID.setD(d);

      // rightWheelEncoderPID.setPID (p, i, d);
      // rightWheelEncoderPID.setP(p);
      // rightWheelEncoderPID.setI(i);
      // rightWheelEncoderPID.setD(d);

      talonLeft.config_kP(0, p);
      talonLeft.config_kI(0, i);
      talonLeft.config_kD(0, d);

      talonRight.config_kP(0, p);
      talonRight.config_kI(0, i);
      talonRight.config_kD(0, d);
    }
  }

  /*
   * public void setBothWheelEncoders(double p, double i, double d) {
   * leftWheelEncoderPID.setPID (p, i, d); leftWheelEncoderPID.setP(p);
   * leftWheelEncoderPID.setI(i); leftWheelEncoderPID.setD(d);
   * leftWheelEncoderPID.setF(f);
   * 
   * rightWheelEncoderPID.setPID (p, i, d); rightWheelEncoderPID.setP(p);
   * rightWheelEncoderPID.setI(i); rightWheelEncoderPID.setD(d);
   * rightWheelEncoderPID.setF(f); }
   */

  public void disableGyroPID() {
    if (Constants.wristEnabled) {
      gyroPID.disable();
    }
  }

  public void setLeft(double speed) {
    if (Constants.wristEnabled) {
      talonLeft.set(ControlMode.PercentOutput, speed);
      // for (VictorSPX i : victorsLeft)
      // i.set(ControlMode.PercentOutput, Math.min(Math.max(speed, -1), 1));
    }
  }

  public void setRight(double speed) {
    if (Constants.wristEnabled) {
      talonRight.set(ControlMode.PercentOutput, speed);
      // for (VictorSPX i : victorsRight)
      // i.set(ControlMode.PercentOutput, Math.max(Math.min(-speed, -1), 1));
    }
  }

  public void setLeftMotorPosition(double position) {
    if (Constants.wristEnabled) {
      talonLeft.set(ControlMode.Position, position);
    }
  }

  public void setRightMotorPosition(double position) {
    if (Constants.wristEnabled) {
      talonRight.set(ControlMode.Position, position);
    }
  }

  public void setBothPositions(double talonLeftPosition, double talonRightPosition) {
    if (Constants.wristEnabled) {
      setBothPositions(talonLeftPosition, talonRightPosition, 0);
    }
  }

  public void setBothPositions(double talonLeftPosition, double talonRightPosition, double gyroCorrection) {
    if (Constants.wristEnabled) {
      talonLeftPosition *= Constants.ticksOfEncoder / Constants.circumferenceOfWheels;
      talonRightPosition *= Constants.ticksOfEncoder / Constants.circumferenceOfWheels;
      setLeftMotorPosition(talonLeftPosition);
      setRightMotorPosition(talonRightPosition);
      // talonRight.set(ControlMode.Position, talonRightPosition, DemandType.AuxPID,
      // gyroCorrection);
    }
  }

  public double getWheelDistanceLeft() {
    if (Constants.wristEnabled) {
      return talonLeft.getSelectedSensorPosition();
    } else {
      return 0;
    }
  }

  public double getWheelDistanceRight() {
    if (Constants.wristEnabled) {
      return talonRight.getSelectedSensorPosition();
    } else {
      return 0;
    }
  }

  public void setBoth(double speedLeft, double speedRight) {
    if (Constants.wristEnabled) {
      setLeft(speedLeft);
      setRight(speedRight);
    }
  }

  public void setBoth(double speed) {
    if (Constants.wristEnabled) {
      setLeft(speed);
      setRight(speed);
    }
  }

  public void setTalonMotorPositions(double position) {
    if (Constants.wristEnabled) {
      talonLeft.set(ControlMode.Position, position / Constants.circumferenceOfWheels * Constants.ticksOfEncoder);
      talonRight.set(ControlMode.Position, position / Constants.circumferenceOfWheels * Constants.ticksOfEncoder);
    }
  }

  public void resetEncoder() {
    if (Constants.wristEnabled) {
      talonLeft.setSelectedSensorPosition(0);
      talonRight.setSelectedSensorPosition(0);
    }
  }

  public void resetSensorPositions() {
    if (Constants.wristEnabled) {
      talonLeft.setSelectedSensorPosition(0);
      talonRight.setSelectedSensorPosition(0);
    }
  }

  public void calibrateGyro() {
    if (Constants.wristEnabled) {
      gyroDrive.calibrate();
    }
  }

  public double getAngle() {
    if (Constants.wristEnabled) {
      return gyroDrive.getAngle() % 360;
    } else {
      return 0;
    }
  }

  public int getEncoderLeft() {
    if (Constants.wristEnabled) {
      return talonLeft.getSelectedSensorPosition();
    } else {
      return 0;
    }
  }

  public int getEncoderRight() {
    if (Constants.wristEnabled) {
      return talonRight.getSelectedSensorPosition();
    } else {
      return 0;
    }
  }

  public void setdrivePIDValues(double p, double i, double d) {
    if (Constants.wristEnabled) {
      // fourBarPID.setPID(p, i, d);
      talonLeft.config_kP(0, p);
      talonLeft.config_kI(0, i);
      talonLeft.config_kD(0, d);
      // fourBarPID.setP(p);
      // fourBarPID.setI(i);
      // fourBarPID.setD(d);
    }
  }

  public void setdrivePIDValues(double p, double i, double d, double f) {
    if (Constants.wristEnabled) {
      setdrivePIDValues(p, i, d);
      talonLeft.config_kF(0, f);
      // fourBarPID.setP(p);
      // fourBarPID.setI(i);
      // fourBarPID.setD(d);
      // fourBarPID.setF(f);
    }
  }

  public double getRightSensorPosition() {
    if (Constants.wristEnabled) {
      return talonRight.getSelectedSensorPosition();
    } else {
      return 0;
    }
  }

  public double getRightError() {
    if (Constants.wristEnabled) {
      return talonRight.getClosedLoopError();
    } else {
      return 0;
    }
  }

  public boolean getLeftSensor() {
    if (Constants.wristEnabled) {
      return leftSensor.get();
    } else {
      return false;
    }
  }

  // sets leftSensor to a boolean value of true being it is seeing the tape
  public boolean getFrontSensor() {
    if (Constants.wristEnabled) {
      return frontSensor.get();
    } else {
      return false;
    }
  }

  // sets frontSensor to a boolean value of true being it is seeing the tape
  public boolean getRightSensor() {
    if (Constants.wristEnabled) {
      return rightSensor.get();
    } else {
      return false;
    }
  }

  public boolean getBackSensor() {
    if (Constants.wristEnabled) {
      return backSensor.get();
    } else {
      return false;
    }
  }

  public void initDefaultCommand() {
    if (Constants.wristEnabled) {
      setDefaultCommand(new TankDriveCommand());
    }
  }

}
