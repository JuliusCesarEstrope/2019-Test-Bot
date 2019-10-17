
package frc.robot.autonomi;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AutonInput extends CommandGroup{

  private static boolean onLevel2, crossField, invertTurns, targetMiddle, startMiddle, NEI; //NEI = not enough information
  private static double distanceToTarget;
  private static char targetSide; //'l' for left, 'm' for mid, and 'r' for right
  
  public AutonInput(){

    onLevel2 = SmartDashboard.getBoolean("Start Level 2", false); //onPlatform
    startMiddle = false;
    crossField = false;
    invertTurns = false;
    targetMiddle = false;
    NEI = false;

    if(SmartDashboard.getBoolean("Back Left", false)/*targetLeftFar*/){
      distanceToTarget = Constants.setpointToFar; //add distance from setpoint to far left
      targetSide = 'l';
    } else if(SmartDashboard.getBoolean("Middle Left", false)/*targetLeftMid*/){
      distanceToTarget = Constants.setpointToMid; //add distance from setpoint to mid left
      targetSide = 'l';
    } else if(SmartDashboard.getBoolean("Front Left", false)/*targetLeftClose*/){
      distanceToTarget = Constants.setpointToClose; //add distance from setpoint to close left
      targetSide = 'l';
    } else if(SmartDashboard.getBoolean("Back Right", false)/*targetRightFar*/){
      distanceToTarget = Constants.setpointToFar; //add distance from setpoint to far right
      targetSide = 'r';
    } else if(SmartDashboard.getBoolean("Middle Right", false)/*targetRightMid*/){
      distanceToTarget = Constants.setpointToMid; //add distance from setpoint to mid right
      targetSide = 'r';
    } else if(SmartDashboard.getBoolean("Front Right", false)/*targetRightClose*/){
      distanceToTarget = Constants.setpointToClose; //add distance from setpoint to close right
      targetSide = 'r';
    } else if(SmartDashboard.getBoolean("Left Front", false)/*targetMidLeft*/){
      distanceToTarget = 0;
      targetSide = 'm';
    } else if(SmartDashboard.getBoolean("Right Front", true)/*targetMidRight*/){
      distanceToTarget = 1;
      targetSide = 'm';
    } else NEI = true;

    if(SmartDashboard.getBoolean("Start Left", true) /*startLeft*/){

      if(targetSide == 'm')
        targetMiddle = true;
      else if(targetSide == 'r')
        crossField = true;

    } else if(SmartDashboard.getBoolean("Start Middle", false) /*startMid*/){

      startMiddle = true;

      if(targetSide == 'm')
        targetMiddle = true;
      else if(targetSide == 'l')
        crossField = true;

    } else if(SmartDashboard.getBoolean("Start Right", false) /*startRight*/){

      invertTurns = true;

      if(targetSide == 'm')
        targetMiddle = true;
      else if(targetSide == 'l')
        crossField = true;

    } else NEI = true;

    if(NEI){
      //auton = new DoNothingCommand();
    }else{
      addSequential(new Auton(onLevel2, startMiddle, invertTurns, targetMiddle, crossField, distanceToTarget));
    }

  }

}