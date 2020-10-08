package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class Robot extends TimedRobot {
    public static double TX, TY;
    private static OI oi;
    private static dashboard6369 d;
    public static DigitalInput limit = new DigitalInput(0);

    public static TalonSRX control_panel = new TalonSRX(RobotMap.CONTROL_PANEL);
    // Game Data
    private String gameData;

    // Camera
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");   
    //CameraServer server;

    //test
    double rotation;
    private double tx_1, tx_2;

    private static DrivetrainSubsystem drivetrain;

    public Timer time;
    public boolean stateFlag, shootState, shooterSpeedState;
    public double offsetTrench = 0;
    public double offset = 0.025;
    public boolean secondAuto;

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        oi = new OI();
        drivetrain = DrivetrainSubsystem.getInstance();
        d = new dashboard6369();
        subSystem.getInstance().configshooter();
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(320, 240);
        //camera.setFPS(20);
        stateFlag = false;
        shootState = false;
        //VideoSink source = CameraServer.getInstance().addSwitchedCamera("Switched Camera");
        //source.setSource(camera);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // 0 is normal vision , 3 is lights on
        secondAuto = false;
    }

    @Override
    public void robotPeriodic() {
        TX = tx.getDouble(0.0);
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
        Scheduler.getInstance().run();
        // Camera
        TY = ty.getDouble(0.0);
        dashboard6369.tx.setDouble(tx.getDouble(0.0));
        dashboard6369.ty.setDouble(ty.getDouble(0.0));
        dashboard6369.ta.setDouble(ta.getDouble(0.0));
        dashboard6369.limit.setBoolean(limit.get());
        if (dashboard6369.Auto.getDouble(0) == 1)
            secondAuto = true;
        else
            secondAuto = false;
        if (dashboard6369.dzero.getBoolean(false)){
            //DrivetrainSubsystem.getInstance().zeroDriveTrain();
        }
        //gameData = DriverStation.getInstance().getGameSpecificMessage();
        //SmartDashboard.putString("Color", gameData);
        SmartDashboard.putNumber("Distance", subSystem.getInstance().distance_estimator(TY));
    }

    public void autonomousInit(){
    }

    public void autonomousPeriodic(){
        if (!secondAuto){    
            if (stateFlag == false){
            stateFlag = true;
            time = new Timer();
            time.start();
            }
            if(time.get() <= 1.0)
                DrivetrainSubsystem.getInstance().drive(new Translation2d(-0.5, 0), 0, true);
            if((time.get() == 1.0))
                DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), 0, true);
            if((time.get() > 1.0) && (time.get() <= 3)){
                    // (DAN CHANGED 3/4/2020) NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // 3 is led vision
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
                    DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), subSystem.getInstance().autoAllignment(), true);
            }
            if((time.get() > 3) && (time.get() <= 15)) {
                subSystem.shooter.set(ControlMode.Velocity, 67000);
                if(subSystem.shooter.getSelectedSensorVelocity() >= 55000)
                    shootState = true;
                if (shootState == true){
                    subSystem.hopperMotor.set(ControlMode.PercentOutput, -0.5);
                    subSystem.feeder.set(ControlMode.PercentOutput, 0.5);
                }
            }
            if(time.get() > 15){
                subSystem.hopperMotor.set(ControlMode.PercentOutput, 0);
                subSystem.shooter.set(ControlMode.PercentOutput, 0);
            }
        }
        else{
            if(time.get() <= 1.0)
                DrivetrainSubsystem.getInstance().drive(new Translation2d(-0.5, 0), 0, true);
            if((time.get() == 1.0))
                DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), 0, true);
        }
    }

    public void teleopPeriodic(){
       //Scheduler.getInstance().run();
       subSystem.getInstance().shoot();
       subSystem.getInstance().hopper();
       subSystem.getInstance().intake();
       subSystem.getInstance().climber();
       subSystem.getInstance().camera();
       subSystem.getInstance().letMeShoot();
       dashboard6369.cellShootDetect.setBoolean(subSystem.getInstance().cell_shoot_detect());
       //ControlPanel.colorAlign(gameData);
    }
    
    public void testPeriodic(){
        if (RobotMap.primaryJoystick.getRawButton(3))
            subSystem.winch.set(ControlMode.PercentOutput, -0.4);
        else if (RobotMap.primaryJoystick.getRawButton(2))
            subSystem.winch.set(ControlMode.PercentOutput, 0.4);
        else
            subSystem.winch.set(ControlMode.PercentOutput, 0);
        if (RobotMap.primaryJoystick.getRawButton(8))
            subSystem.hook_lift.set(ControlMode.PercentOutput, 0.6);
        else if (RobotMap.primaryJoystick.getRawButton(7))
            subSystem.hook_lift.set(ControlMode.PercentOutput, -0.18);
        else
            subSystem.hook_lift.set(ControlMode.PercentOutput, 0);
    }
}
