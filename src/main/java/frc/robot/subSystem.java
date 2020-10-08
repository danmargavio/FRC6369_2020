package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subSystem { // super-class for driveBase

    public static ButtonDebouncer hopperintake_inDebouncer, hopperintake_outDebouncer, climberDebouncer;
    public static TalonSRX intake, shooter, shooter2, winch;
    public static VictorSPX hook_lift, hopperMotor, feeder;
    public static Servo stop;
    public static DoubleSupplier per;
    private double vel1, time1, increment_lift, increment_winch;
    private static subSystem instance;


    public subSystem(){
        hopperMotor = new VictorSPX(RobotMap.HOPPER);
        feeder = new VictorSPX(RobotMap.CONTROL_PANEL);
        intake = new TalonSRX(RobotMap.INTAKE);
        shooter = new TalonSRX(RobotMap.SHOOTER);
        shooter2 = new TalonSRX(RobotMap.SHOOTER2);
        hook_lift = new VictorSPX(RobotMap.HOOK_LIFT);
        winch = new TalonSRX(RobotMap.WINCH);
        stop = new Servo(0);
        vel1 = shooter.getSelectedSensorVelocity();
        time1 = Timer.getFPGATimestamp();
        increment_lift = 0;
        increment_winch = 0;
    }

    public static subSystem getInstance() {
        if (instance == null) {
            instance = new subSystem();
        }
        return instance;
    }

    public void hopper(){
        if (RobotMap.secondaryJoystick.getRawAxis(3) > 0.08)
            hopperMotor.set(ControlMode.PercentOutput, -RobotMap.secondaryJoystick.getRawAxis(3));
        else if (RobotMap.secondaryJoystick.getRawAxis(2) > 0.08)
            hopperMotor.set(ControlMode.PercentOutput, RobotMap.secondaryJoystick.getRawAxis(2));
        else
            hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    public void intake(){
        if (RobotMap.primaryJoystick.getRawAxis(3) > 0.08)
            intake.set(ControlMode.PercentOutput, -0.90*RobotMap.primaryJoystick.getRawAxis(3));
        else if (RobotMap.primaryJoystick.getRawAxis(2) > 0.08)
            intake.set(ControlMode.PercentOutput, 0.90*RobotMap.primaryJoystick.getRawAxis(2));
        else
            intake.set(ControlMode.PercentOutput, 0);
    }

    public void camera(){
        if (RobotMap.secondaryJoystick.getPOV() == 0) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
        }
        else if (RobotMap.secondaryJoystick.getPOV() == 180) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        }
        else if (RobotMap.secondaryJoystick.getPOV() == -1) {
            // no change
        }
    }

    public void configshooter(){
        shooter2.set(ControlMode.Follower, RobotMap.SHOOTER);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shooter.setSelectedSensorPosition(0);
        shooter.setSensorPhase(false);
        shooter.setInverted(true);
        shooter.config_kF(0, 0, 10);
		shooter.config_kP(0, 0.14, 10);
		shooter.config_kI(0, 0, 10);
        shooter.config_kD(0, 0, 10);
        shooter.configClosedLoopPeakOutput(0, 0.90);
        shooter2.configClosedLoopPeakOutput(0, 0.90);
    }

    public void shoot(){
        if (RobotMap.primaryJoystick.getRawButton(4)){
            shooter.set(ControlMode.Velocity, 70000); //75000 was used previously
            //shooter.set(ControlMode.PercentOutput, 0.80);
        }
        //else if (RobotMap.con0.getRawAxis(2) >= 0.05 && RobotMap.con0.getRawAxis(3) <= 0.05){
            //shooter.set(ControlMode.PercentOutput, -RobotMap.con0.getRawAxis(2) * 0.75);
        //}
        else {
            shooter.set(ControlMode.PercentOutput, 0);
        }
        //SmartDashboard.putNumber("shooter percent", dashboard6369.shooterPercent.getDouble(1.0));
        //if(dashboard6369.develop.getDouble(-9999) == 1.0)
          //  dashboard6369.shooterPercent.setDouble(1000000000);

        // ======== (3/4/2020) Dan added code to below to indicate that shooter speed has reached acceptable velocity and send to dashboard
        if (shooter.getSelectedSensorVelocity() > 60000) {
            SmartDashboard.putBoolean("shoot ready", true);
        }else{
            SmartDashboard.putBoolean("shoot ready", false);
        }
        // ======== (3/4/2020) Dan added code above to indicate that shooter speed has reached acceptable velocity and send to dashboard

        //SmartDashboard.putNumber("shooter velocity", shooter.getSelectedSensorVelocity());
        dashboard6369.shootVelocity.setDouble(shooter.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("shooter current", shooter.getSupplyCurrent());
        dashboard6369.tx.setDouble(dashboard6369.shooterPercent.getDouble(1.0));
        dashboard6369.shootCurrent.setDouble(shooter.getSupplyCurrent());
        //SmartDashboard.putNumber("shooter voltage", shooter.getMotorOutputVoltage());
        dashboard6369.shootVoltage.setDouble(shooter.getMotorOutputVoltage());
        //SmartDashboard.putNumber("shooter2 voltage", shooter2.getMotorOutputVoltage());
        dashboard6369.shoot2Voltage.setDouble(shooter2.getMotorOutputVoltage());
        //SmartDashboard.putNumber("shooter2 current", shooter2.getSupplyCurrent());
        dashboard6369.shoot2Current.setDouble(shooter2.getSupplyCurrent());
        //SmartDashboard.putNumber("shooter position", shooter.getSelectedSensorPosition());
        //if (shooter.getSelectedSensorVelocity() >= 35000)
            //stop.set(0.50);
        dashboard6369.shootAcceleration.setDouble(acceleration());
    }

    public double acceleration(){
        double acc;
        acc = (shooter.getSelectedSensorVelocity() - vel1) / (Timer.getFPGATimestamp() - time1);
        vel1 = shooter.getSelectedSensorVelocity();
        time1 = Timer.getFPGATimestamp();
        return acc;
    }

    public boolean cell_shoot_detect(){
        if (acceleration() > 6000)
            return true;
        else
            return false;
    }

    public void letMeShoot(){
        if (RobotMap.secondaryJoystick.getRawButton(5)){
            feeder.set(ControlMode.PercentOutput, 0.9);
        }else if (RobotMap.secondaryJoystick.getRawButton(6)){
            feeder.set(ControlMode.PercentOutput, -0.9);
        }else
            feeder.set(ControlMode.PercentOutput, 0);
    }

    public void climber(){
        if (RobotMap.primaryJoystick.getRawButton(8)){
            increment_lift = dashboard6369.lift_power.getDouble(0.95);
        }else if (RobotMap.primaryJoystick.getRawButton(7)){
            increment_lift = -0.9;
        }else{
            increment_lift = 0;
        }
        if (RobotMap.primaryJoystick.getRawButton(9) && RobotMap.secondaryJoystick.getRawButton(7)){
            if (RobotMap.primaryJoystick.getRawButton(2)){
                increment_winch = 0.6;
            }
        }else{
            increment_winch = 0;
        }
        winch.set(ControlMode.PercentOutput, increment_winch);
        hook_lift.set(ControlMode.PercentOutput, increment_lift);
    }

    public double autoAllignment(){
        double offset = dashboard6369.offset.getDouble(0);
        double tx_1 = Robot.TX; // make a local copy of the tx value
        double tx_2 = (tx_1/29.8) - offset;  // scale the 2nd tx value by the total range of tx (-27 to +27 degrees)
        double rotation;
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
        }
        return rotation;
    }

    /**
     * This function takes the limelight camera ty value in degrees and outputs the estimated distance to the robot camera in inches
     * @param ty_angle
     * @return distance
     */
    public double distance_estimator(double ty_angle){
        return 62/Math.tan(Math.PI * (8.686 + ty_angle) / 180);
    }
}