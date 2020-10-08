package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
public class RobotMap {

    public static Joystick primaryJoystick = new Joystick(0);
    public static Joystick secondaryJoystick = new Joystick(1);

    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 16; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 17; // CAN

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 21; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 20; // CAN

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 15; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 14; // CAN

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 18; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 19; // CAN

    public static final int SHOOTER = 8; // CAN, has encoder 40 amp
    public static final int SHOOTER2 = 9;// 40 amp
    public static final int FEEDER = 2; //CAN 20 amp NOT USED ANYMORE
    public static final int HOPPER = 3; //CAN 20 amp
    public static final int INTAKE = 4; //CAN 40 amp
    public static final int WINCH = 5; //CAN 40 amp
    public static final int HOOK_LIFT = 6; //CAN, 40 amp has encoder
    public static final int CONTROL_PANEL = 7; //CAN, 20 amp
}
