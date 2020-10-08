package com.swervedrivespecialties.exampleswerve.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {
    private double tx_1, tx_2, rotation, slow;
    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
      if (RobotMap.primaryJoystick.getRawButton(6))
        slow = 0.2;
      else
        slow = 1;
      double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
      forward = Utilities.deadband(forward);
      // Square the forward stick
      forward = Math.copySign(Math.pow(forward, 2.0), forward) * slow;

      double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
      strafe = Utilities.deadband(strafe);
      // Square the strafe stick
      strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * slow;
      //double offset = dashboard6369.offset.getDouble(0);
      if (RobotMap.secondaryJoystick.getRawButton(8)){
          //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
          //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // 3 is led vision
          /*tx_1 = Robot.TX;   // make a local copy of the tx value
          tx_2 = (tx_1/27.0) - offset;  // scale the 2nd tx value by the total range of tx (-27 to +27 degrees)
          if (Math.abs(tx_2) > 0.50) {
            rotation = -0.2 * Math.signum(tx_2);
          }
          else if (Math.abs(tx_2) < 0.50  && Math.abs(tx_2) >= 0.20) {
            rotation = -0.1 * Math.signum(tx_2);        
          }  // create a minimum control band so that the rotation command doesnt become too small
          else if (Math.abs(tx_2) < 0.20 && Math.abs(tx_2) >= 0.02) {  // arbitrarily set a tolerable tx error band of 2%
            rotation = -0.02 * Math.signum(tx_2);
          }
          else {
            rotation = 0;
          }*/
        rotation = subSystem.getInstance().autoAllignment();
      }else {
          //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        rotation = -RobotMap.primaryJoystick.getRawAxis(4);
        rotation = Utilities.deadband(rotation);
          // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
      }
      DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
      //DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), rotation, true);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
}
