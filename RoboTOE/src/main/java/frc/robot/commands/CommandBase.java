package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.BooperSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.WristSubsystem;

public abstract class CommandBase extends Command {

  public static OI oi;
  public static DriveSubsystem drive;
  public static WristSubsystem wrist;
  public static BooperSubsystem booper;
  public static RollerSubsystem roller;
  public static double turnAngle;

  public static void init() {

    drive = new DriveSubsystem(Constants.motorPortsLeft, Constants.motorPortsRight, Constants.gyroPort, Constants.driveEncoderPortLeft, Constants.driveEncoderPortRight, Constants.frontSensor, Constants.rightSensor, 
    Constants.backSensor, Constants.leftSensor, Constants.driveEncoderPortLeft, Constants.driveEncoderPortRight, Constants.circumferenceOfWheels, Constants.ticksOfEncoder, Constants.driveRotationPIDValues);
    roller = new RollerSubsystem(Constants.rollerMotor);
    booper = new BooperSubsystem(Constants.booperPorts);
    wrist = new WristSubsystem(Constants.leftWristMotorPort, Constants.rightWristMotorPort, Constants.leftWristEncoderPort, Constants.rightWristEncoderPort, Constants.wristPIDValues);
    oi = new OI();
    //led.setLEDLightColor(.87); // Blue
    wrist.ResetEncoder();
    
  }

  public static void disable() {

  }
}
