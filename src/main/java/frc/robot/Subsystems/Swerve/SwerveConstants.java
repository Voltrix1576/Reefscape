package frc.robot.Subsystems.Swerve;

public class SwerveConstants {

  // swerve constants
    public final static double WIDTH = 0.53; 
    public final static double LENGTH = 0.53;
    public final static double RADIUS = Math.sqrt(Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;

    // Modules constants
    public final static double TURNING_GEAR_RATIO = 12.8;
    private final static double DRIVE_GEAR_RATIO = 6.75;
    private final static double WHEEL_RADIUS = 0.0508;

    public final static double DISTANCE_PER_PULSE = (2 * WHEEL_RADIUS * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double POSE_TURNING_FACTOR = (360 / TURNING_GEAR_RATIO);

    // drive pid
    public static final double DRIVE_KV = 2.4;
    public static final double DRIVE_KS = 0.15;
    public static final double DRIVE_KP = 0.001;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;

    // turning pid
    public static final double TURNING_KP = 0.06;
    public static final double TURNING_KI = 0;
    public static final double TURNING_KD = 0;

    //swerve physics
    public static final double MAX_VELOCITY = 4.572;
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;

    public static final double MAX_ACCELERATION = Math.pow(MAX_VELOCITY, 2) / RADIUS;
    public static final double MAX_ANGOLAR_ACCELERATION = MAX_ACCELERATION / RADIUS;

    //front left module
    public static final int FRONT_LEFT_DRIVE_ID = 5;
    public static final int FRONT_LEFT_TURNING_ID = 12;
    public static final int FRONT_LEFT_ABSULOTE_ENCODER_ID = 7; 
    public static final double FRONT_LEFT_ABSULOTE_ENCODER_OFFSET = 130.869;
    public static final boolean FRONT_LEFT_IS_DRIVE_MOTOR_INVERTED  = true;  
    public static final boolean FRONT_LEFT_IS_TURNING_MOTOR_INVERTED = true;
    public static final boolean FRONT_LEFT_IS_ABSULOTE_ENCODER_INVERTED = true;

    //front right module
    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_TURNING_ID = 13;
    public static final int FRONT_RIGHT_ABSULOTE_ENCODER_ID = 8; 
    public static final double FRONT_RIGHT_ABSULOTE_ENCODER_OFFSET = 52.998;
    public static final boolean FRONT_RIGHT_IS_DRIVE_MOTOR_INVERTED  = true;  
    public static final boolean FRONT_RIGHT_IS_TURNING_MOTOR_INVERTED = true;
    public static final boolean FRONT_RIGHT_IS_ABSULOTE_ENCODER_INVERTED = true;

    //rear left module
    public static final int REAR_LEFT_DRIVE_ID = 4;
    public static final int REAR_LEFT_TURNING_ID = 11;
    public static final int REAR_LEFT_ABSULOTE_ENCODER_ID = 10; 
    public static final double REAR_LEFT_ABSULOTE_ENCODER_OFFSET = 344.443;
    public static final boolean REAR_LEFT_IS_DRIVE_MOTOR_INVERTED  = true;   
    public static final boolean REAR_LEFT_IS_TURNING_MOTOR_INVERTED = true;
    public static final boolean REAR_LEFT_IS_ABSULOTE_ENCODER_INVERTED = true;

    //rear right module
    public static final int REAR_RIGHT_DRIVE_ID = 6;
    public static final int REAR_RIGHT_TURNING_ID = 14;
    public static final int REAR_RIGHT_ABSULOTE_ENCODER_ID = 9; 
    public static final double REAR_RIGHT_ABSULOTE_ENCODER_OFFSET = 32.255;
    public static final boolean REAR_RIGHT_IS_DRIVE_MOTOR_INVERTED  = true;  
    public static final boolean REAR_RIGHT_IS_TURNING_MOTOR_INVERTED = true;
    public static final boolean REAR_RIGHT_IS_ABSULOTE_ENCODER_INVERTED = true;
    
}