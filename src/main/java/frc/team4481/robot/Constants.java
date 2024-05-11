package frc.team4481.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.*;
import frc.team4481.lib.feedback.led.pattern.*;
import frc.team4481.lib.swerve.FFValueContainer;
import frc.team4481.lib.swerve.PIDValueContainer;
import frc.team4481.robot.util.ShamperSpeeds;

/*
* IMPORTANT:
* If you want to switch between rev and sds swerve modules do CTRL+F and type SDS
* then you will find the places where some constants need to be changed
* */

public class Constants {
    /* ---------------------------------------- */
    /* LOOPER */
    /* ---------------------------------------- */
    public static final double kLooperDt = 0.01;
    public static double kHIDLooperDt = 0.02;

    public enum SWERVE_MODULE_TYPE {
        REV_NEO_NEO550(0),
        SDS_NEO_NEO(1),
        REV_VORTEX_NEO550(2);

        public final int index;

        SWERVE_MODULE_TYPE(int index) {
            this.index = index;
        }
    }

    // IMPORTANT: change this value based on swerve module type
    public static final SWERVE_MODULE_TYPE MODULE_TYPE = SWERVE_MODULE_TYPE.REV_VORTEX_NEO550;
    /**
     * Static class that holds all the CAN IDs and IO port addresses on the RoboRIO
     */
    public static class HardwareMap {
        // Sensors
        public static final int PIGEON_IMU = new int[]{9, 15, 9}[MODULE_TYPE.index];

        // Drivetrain
        public static final int DT_FRONT_LEFT_DRIVE_ID = new int[]{11, 21, 11}[MODULE_TYPE.index];
        public static final int DT_FRONT_LEFT_TURN_ID = new int[]{12, 22, 12}[MODULE_TYPE.index];
        public static final boolean DT_FRONT_LEFT_INVERTED = new boolean[]{true, false, true}[MODULE_TYPE.index];
        public static final int DT_FRONT_LEFT_TURN_ENCODER_ID = new int[]{-1, 13, -1}[MODULE_TYPE.index];
        public static final double DT_FRONT_LEFT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 105, Double.NaN}[MODULE_TYPE.index];

        public static final int DT_FRONT_RIGHT_DRIVE_ID = new int[]{13, 23, 13}[MODULE_TYPE.index];
        public static final int DT_FRONT_RIGHT_TURN_ID = new int[]{14, 24, 14}[MODULE_TYPE.index];
        public static final boolean DT_FRONT_RIGHT_INVERTED = new boolean[]{false, true, false}[MODULE_TYPE.index];
        public static final int DT_FRONT_RIGHT_TURN_ENCODER_ID = new int[]{-1, 12, -1}[MODULE_TYPE.index];
        public static final double DT_FRONT_RIGHT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 32, Double.NaN}[MODULE_TYPE.index];

        public static final int DT_BACK_LEFT_DRIVE_ID = new int[]{15, 25, 15}[MODULE_TYPE.index];
        public static final int DT_BACK_LEFT_TURN_ID = new int[]{16, 26, 16}[MODULE_TYPE.index];
        public static final boolean DT_BACK_LEFT_INVERTED = new boolean[]{true, false, true}[MODULE_TYPE.index];
        public static final int DT_BACK_LEFT_TURN_ENCODER_ID = new int[]{-1, 11, -1}[MODULE_TYPE.index];
        public static final double DT_BACK_LEFT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 1, Double.NaN}[MODULE_TYPE.index];

        public static final int DT_BACK_RIGHT_DRIVE_ID = new int[]{17, 27, 17}[MODULE_TYPE.index];
        public static final int DT_BACK_RIGHT_TURN_ID = new int[]{18, 28, 18}[MODULE_TYPE.index];
        public static final boolean DT_BACK_RIGHT_INVERTED = new boolean[]{false, true, false}[MODULE_TYPE.index];
        public static final int DT_BACK_RIGHT_TURN_ENCODER_ID = new int[]{-1, 14, -1}[MODULE_TYPE.index];
        public static final double DT_BACK_RIGHT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 238, Double.NaN}[MODULE_TYPE.index];

        public static final int[] DRIVE_IDS =
                { DT_FRONT_LEFT_DRIVE_ID, DT_FRONT_RIGHT_DRIVE_ID, DT_BACK_LEFT_DRIVE_ID, DT_BACK_RIGHT_DRIVE_ID };
        public static final int[] TURN_IDS =
                { DT_FRONT_LEFT_TURN_ID, DT_FRONT_RIGHT_TURN_ID, DT_BACK_LEFT_TURN_ID, DT_BACK_RIGHT_TURN_ID };
        public static final boolean[] INVERTED =
                { DT_FRONT_LEFT_INVERTED, DT_FRONT_RIGHT_INVERTED, DT_BACK_LEFT_INVERTED, DT_BACK_RIGHT_INVERTED };
        public static final int[] TURN_ENCODER_IDS =
                {DT_FRONT_LEFT_TURN_ENCODER_ID, DT_FRONT_RIGHT_TURN_ENCODER_ID, DT_BACK_LEFT_TURN_ENCODER_ID, DT_BACK_RIGHT_TURN_ENCODER_ID};
        public static final double[] TURN_ENCODER_OFFSET_DEGREES =
                {DT_FRONT_LEFT_TURN_ENCODER_OFFSET, DT_FRONT_RIGHT_TURN_ENCODER_OFFSET, DT_BACK_LEFT_TURN_ENCODER_OFFSET, DT_BACK_RIGHT_TURN_ENCODER_OFFSET};

        //Intake
        public static final int IN_MOTOR_ID = 21;
        public static final int STORAGE_MOTOR_ID = 22;
        public static final int FULL_WIDTH_ROLLER_ID = 23;
        public static final int LOW_SENSOR_ID = 0;
        public static final int HIGH_SENSOR_ID = 1;

        // Pivot
        public static final int PIVOT_CAN_ID = 31;


        // Shamper
        public static final int FEEDER_SHOOTER_CAN_ID = 32;
        public static final int KICKER_SHOOTER_CAN_ID = 33;
        public static final int TOP_SHOOTER_CAN_ID = 34;
        public static final int BOTTOM_SHOOTER_CAN_ID = 35;
        public static final int SHOOT_SENSOR_ID = 2;


        // Climber
        public static final int CLIMBER_LEFT_ID = 41;
        public static final int CLIMBER_RIGHT_ID = 42;



    }

    /**
     * Static class containing all the constants for the drivetrain subsystem
     */
    public static class Drivetrain {
        /**
         * 1-dimensional distance between the center of a wheel and the center of the drivetrain in m
         */
        public static final double DRIVETRAIN_WHEELBASE_DISTANCE = new double[]{0.285, 0.45 / 2, 0.5111/2}[MODULE_TYPE.index];
        public static final double DRIVETRAIN_WIDTH = new double[]{0.79, 0.82, 0.79}[MODULE_TYPE.index];
        public static final double DRIVE_GEAR_RATIO = new double[]{4.71, 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0)), 4.29}[MODULE_TYPE.index];
        public static final double WHEEL_RADIUS = new double[]{0.0760 / 2, 0.1016 / 2, 0.07515 / 2}[MODULE_TYPE.index];
        public static final double TURN_GEAR_RATIO = new double[]{2.89 * 3.61 /14 * 62, (15.0 / 32.0) * (10.0 / 60.0), 2.89 * 3.61 /14 * 62}[MODULE_TYPE.index];


        public static final int STALL_LIMIT_DRIVE = 65; //Amps
        public static final int STALL_LIMIT_TURN = 23; //in Amps

        public static final Translation2d FRONT_LEFT_LOCATION =
                new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d FRONT_RIGHT_LOCATION =
                new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d BACK_LEFT_LOCATION =
                new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d BACK_RIGHT_LOCATION =
                new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);

        /**
         *  Minimum limit for the robot heading slew rate limiter in RAD/S
         *  The used limit depends on the current speed of the robot
         */
        public static final double MIN_DIRECTION_RATE_LIMIT = 250 / (180/Math.PI);
        /**
         * At what angle change the robot should reverse instead of turn
         */
        public static final Rotation2d REVERSE_THRESHOLD = Rotation2d.fromRadians(0.7 * Math.PI);

        /**
         * Margin that the heading correction algorithm has in degrees
         */
        public static final double HEADING_CORRECTION_MARGIN = 4;
        //P constant for heading correction
        public static final double HEADING_CORRECTION_KP = 0.05;
        //Time to wait before the heading correction to turn on in seconds
        public static final double HEADING_CORRECTION_TURNON_TIME = 0.5;

        /**
         * Drive motor RPM to wheel velocity in m/s
         */
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO * 60);
        /**
         * Drive motor revolutions to distance in m
         */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO);
        /**
         * Turn motor RPM to module angular velocity in rad/s
         */
        public static final double TURN_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / (TURN_GEAR_RATIO * 60);
        /**
         * Turn motor rotations to radians
         */
        public static final double TURN_POSITION_CONVERSION_FACTOR = 2 * Math.PI / TURN_GEAR_RATIO;


        public static final double DRIVE_kP = new double[]{0.2, 0.2, 0.001 * DRIVE_VELOCITY_CONVERSION_FACTOR}[MODULE_TYPE.index];
        public static final double DRIVE_kI = 0;
        public static final double DRIVE_kD = 0;

        public static final double DRIVE_kA = 0; //Currently not used
        public static final double DRIVE_kS = new double[]{0.175, 0.175, 0.19}[MODULE_TYPE.index];
        public static final double DRIVE_kV = new double[]{2.50, 2.95, 2.00}[MODULE_TYPE.index];

        public static final double TURN_kA = 0; //Currently not used
        public static final double TURN_kS = 0;
        public static final double TURN_kV = 0; //Currently not used

        public static final double TURN_kP = 1.0;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = 0;

        public static final PIDValueContainer TURN_PID_VALUES = new PIDValueContainer(TURN_kP, TURN_kI, TURN_kD);
        public static final FFValueContainer TURN_FF_VALUES = new FFValueContainer(TURN_kS, TURN_kV, TURN_kA);
        public static final PIDValueContainer DRIVE_PID_VALUES = new PIDValueContainer(DRIVE_kP, DRIVE_kI, DRIVE_kD);
        public static final FFValueContainer DRIVE_FF_VALUES = new FFValueContainer(DRIVE_kS, DRIVE_kV, DRIVE_kA);

        public static final CANSparkBase.IdleMode DRIVE_IDLE_MODE = CANSparkBase.IdleMode.kBrake;

        public static final Pose2d DRIVETRAIN_START_POSITION_BLUE = new Pose2d();
        public static final Pose2d DRIVETRAIN_START_POSITION_RED = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180));

    }

    public static class Pivot {
        // PIDFF constants
        public static final double PIVOT_KP_AUTOMATIC = 0.075; //kP for automatic mode (shooting at speaker)
        public static final double PIVOT_KP = 0.06;//kP for motion profile mode (amping, etc.)
        public static final double MOTION_PROFILE_DISABLE_MARGIN = 2; //Within which margin the motion profile should be disabled and the pivot will be controlled by only PID
        public static final double MARGIN_SETPOINT_MOVING = 3;
        public static final double MARGIN_SETPOINT_REST = 3;//0.45;
        public static final double MARGIN_SETPOINT_AMPING = 3;
        public static final double DEADZONE_PIVOT_HORIZONTAL = 5; //Within which margin the pivot should be turned off

        public static final double KS = 0.10;
        public static final double KG = 0.55;//0.4;//0.19;//0.40
        public static final double KV = 0.04;//0.04;
        public static final double KA = 0.0015;//0.0015;
        public static final double MAX_VEL = 1500;
        public static final double MAX_ACCEL = 1000;

        // conversion Factor
        public static final double PIVOT_GEAR_RATIO = 1.0/84;

        // conversion factor for encoder
        public static final double PIVOT_POSITION_CONVERSION_FACTOR = 360;
        public static final double PIVOT_ABS_ENC_VEL_CONVERSION_FACTOR = 360;
        public static final double PIVOT_REL_ENC_VEL_CONVERSION_FACTOR = 6;
        public static final double PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET = 30;
        public static final double PIVOT_ABS_ENC_ZERO_OFFSET = 263.7800646 - PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;

        // positions
        public static final double PIVOT_ANGLE_AGAINST_SPEAKER = 48.87 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;
        public static final double PIVOT_ANGLE_PODIUM_SPEAKER = 29 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;
        public static final double PIVOT_ANGLE_AMP = 90 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;
        public static final double PIVOT_ANGLE_INTAKEN = 45 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;
        public static final double PIVOT_ANGLE_STOWED = 1.5 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET; //12;
        public static final double PIVOT_ANGLE_OVER_STAGE = 45 + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;

        // Current limit
        public static final int CURRENT_LIMIT_PIVOT = 70;

    }

    public static class Shamper {
        public static final ShamperSpeeds AGAINST_SUBWOOFER = new ShamperSpeeds(3200,5000,5000);//-5500, 5500
        public static final ShamperSpeeds PODIUM = new ShamperSpeeds(3200,5000,5000);//-5500, 5500
        public static final ShamperSpeeds AMP = new ShamperSpeeds(1000,-3600,750);//3000, 4500, 6000
        public static final ShamperSpeeds INTAKEN = new ShamperSpeeds(-3000,-3000,-3000);
        public static final ShamperSpeeds IDLE = new ShamperSpeeds(0,0,0);
        public static final ShamperSpeeds OVER_STAGE = new ShamperSpeeds(3200, 4800, 4800);
        public static final ShamperSpeeds RAMP_AMP = new ShamperSpeeds( 0,-3000,0);
        public static final ShamperSpeeds RAMP_SPEAKER = new ShamperSpeeds(0,4000,0);
        public static final ShamperSpeeds AUTON_EJECT = new ShamperSpeeds(1000,1000,1000);
        public static final ShamperSpeeds EJECT = new ShamperSpeeds(3500, 6500, 6500);

        public static final double SHAMPER_SETPOINT_MARGIN = 200;
        public static final double SHAMPER_IDLE_MARGIN = 8000;
        public static final double SHAMPER_SETPOINT_MARGIN_AMP = 1400;
        public static final double SHAMPER_KI_MAX_ACCUM = 0.00001;
        public static final double SHAMPER_KI_DEADZONE = 0.0;

        //Current limits
        public static final int CURRENT_LIMIT_KICKER = 50;
        public static final int CURRENT_LIMIT_TOP = 120;
        public static final int CURRENT_LIMIT_BOTTOM = 70;


//      velocity conversionfactor (gear ratio)
        public static final double CONVERSIONFACTOR_KICKER = 11.0/15.0;
        public static final double CONVERSIONFACTOR_TOP_SHOOTER = 1.667;
        public static final double CONVERSIONFACTOR_BOTTOM_SHOOTER = 1.667;

//        PIDF

        public static final double KICKER_KP = 0.00005;
        public static final double KICKER_KI = 0.0000001;
        public static final double KICKER_FF = 1.0/(6600 * CONVERSIONFACTOR_KICKER) * 12;//5700 //1.0/(5750 * CONVERSIONFACTOR_KICKER) * 12;//5850

        public static final double TOP_KP = 0.0003;
        public static final double TOP_KI = 0;// 0.0000001;
        public static final double TOP_FF = 1.0/(6000 * CONVERSIONFACTOR_TOP_SHOOTER) * 12;//5050 //1.0/(4800*CONVERSIONFACTOR_TOP_SHOOTER) * 12;//4400

        public static final double BOTTOM_KP = 0.00004;
        public static final double BOTTOM_KI = 0;//0.000001;
        public static final double BOTTOM_FF = 1.0/(6300 * CONVERSIONFACTOR_BOTTOM_SHOOTER) * 12;//5350 //1.0/(5200 * CONVERSIONFACTOR_BOTTOM_SHOOTER) * 12;//1.0/(5350*CONVERSIONFACTOR_BOTTOM_SHOOTER) * 12;//4600

        public static final int OVERHEAT_THRESOLD = 60;

    }
    public static class Intake{
        //Factor to convert between surface speed of intake wheels and surface speed of storage wheels
        public static final double INTAKE_STORAGE_CONVERSION_FACTOR = 0.85;//0.9375;
        public static final double INTAKE_SPEED = 0.9;
        public static final double INTAKE_SPEED_STORAGE = INTAKE_SPEED * INTAKE_STORAGE_CONVERSION_FACTOR;
        public static final double TURBO_FEEDER_SPEED = 0.8;
        public static final double STORE_SPEED = 0.6;
        public static final double INTAKE_SOURCE_STORAGE_SPEED = -0.4;
        public static final double STORE_SPEED_STORAGE = STORE_SPEED * INTAKE_STORAGE_CONVERSION_FACTOR;
        public static final double FEEDING_SPEED_INTAKE = 0.0;
        public static final double FEEDING_SPEED_STORAGE = 0.8;
        public static final double FEEDING_RPM = 3200;
        public static final double FEEDER_SLOW_SPEED = 100;
        public static final double HIGH_SENSOR_DELAY = 0.04;
        //Gear ratio of feeder
        public static final double CONVERSIONFACTOR_FEEDER = 11.0/30.0;//12.0/32;
        public static final double FEEDER_KP =  0.0;// 0.0002;//0.00001
        public static final double FEEDER_FF = 1.0/(5800 * CONVERSIONFACTOR_FEEDER) * 12;//1.0/(5800 * CONVERSIONFACTOR_FEEDER) * 12;//1.0/(5500*CONVERSIONFACTOR_FEEDER) * 12
        public static final int CURRENT_LIMIT_INTAKE = 70;
        public static final int CURRENT_LIMIT_STORAGE = 50;
        public static final double INTAKE_RAMP_UP_TIME = 0.5;
        public static final double STORAGE_RAMP_UP_TIME = 0.5;
        //Time to use to filter out very short triggers of the intake sensors
        public static final double SENSOR_FILTER_TIME = 0.05;

    }
    public static class Climber{
        public static final double LOW_SETPOINT = 3;
        public static final double HIGH_SETPOINT = 87;
        public static final double MARGE = 0.05;
        public static final int CURRENT_LIMIT = 30;
        public static final double CALIBRATION_CHECK_LIMMIT = 15;
        public static final double CALIBRATION_SPEED = -0.2;
    }

    /**
     * Static class containing important field dimensions
     */
    public static class Field {
        public static final double FIELD_WIDTH = 8.2042;
        public static final double FIELD_LENGTH =  16.5412;
        public static final Translation2d FIELD_UPPER_RIGHT_CORNER = new Translation2d(FIELD_LENGTH, FIELD_WIDTH);
    }

    /* ---------------------------------------- */
    /* PATH PLANNING */
    /* ---------------------------------------- */
    /*
     */

    public static class PathFollowing {
        public static final double MAX_VELOCITY = 5;
        public static final double MAX_TURN_VELOCITY = 2 * Math.PI;

        public static final double DRIVE_CONTROLLER_XY_kP = 3;
        public static final double DRIVE_CONTROLLER_XY_kD = 0;
        public static final double DRIVE_CONTROLLER_THETA_kP = 4;
        //Maximum error of the path in meters, before the adaptive time sampler kicks in
        public static final double MAX_PATH_ERROR = 0.75;
        public static final double ACCEL_FF = 0.25;
    }

    public static class Vision{

        /**
         * Standard deviation of the limelight measurements
         */
        public static final double STANDARD_DEV_TRANSLATION = 0.5;
        public static final double STANDARD_DEV_ROTATION = 1.5;
        public static final double AMBIGUITY_THRESHOLD = 0.17;
        public static final double TAG_AREA_THRESHOLD = 0.06; //0.1;

//        public static final Transform3d CAMERA_FRONT_TRANSFORM = new Transform3d(new Translation3d(0.285, -0.135, 0.256), new Rotation3d(0,-Math.toRadians(30),0));
//        public static final Transform3d CAMERA_BACK_TRANSFORM = new Transform3d(new Translation3d(-0.291, -0.260, 0.318), new Rotation3d(0,-Math.toRadians(25),Math.toRadians(180)));
        //beta
        public static final Transform3d CAMERA_FRONT_TRANSFORM = new Transform3d(new Translation3d(0.263, 0.140, 0.207), new Rotation3d(0,-Math.toRadians(30),Math.toRadians(-25)));
        public static final Transform3d CAMERA_BACK_TRANSFORM = new Transform3d(new Translation3d(0.263, -0.140, 0.207), new Rotation3d(0,-Math.toRadians(30),Math.toRadians(25)));
        public static final String LAYOUT_FILE_NAME = "2024-crescendo";
    }
    public static class AutoAim{
        public static final double SPEAKER_TARGET_X_OFFSET = 0.15; //Offset of the speaker target from the alliance wall (x=0) in meters
        public static final double SPEAKER_APRIL_TAG_RECESS = 0.0381; //How much the April Tag is recessed into the wall (x=0) in meters
        public static final Pose2d SPEAKER_TARGET_POSE_BLUE = new Pose2d(-0.038099999999999995+SPEAKER_TARGET_X_OFFSET,5.547867999999999, new Rotation2d());
        public static final Pose2d SPEAKER_TARGET_POSE_RED = new Pose2d(16.579-SPEAKER_TARGET_X_OFFSET, 5.547867999999999, new Rotation2d());

        public static final Pose2d AMP_SCORE_POSE_BLUE = new Pose2d(1.8915,8.185 - (Drivetrain.DRIVETRAIN_WIDTH/2),Rotation2d.fromDegrees(-90.0));
        public static final Pose2d AMP_SCORE_POSE_RED = new Pose2d(14.650757999999999,8.185 - (Drivetrain.DRIVETRAIN_WIDTH/2),Rotation2d.fromDegrees(-90.0));

        public static final Pose2d SUPERCYLE_TARGET_POSE_BLUE = new Pose2d(                    1.5,Field.FIELD_WIDTH-1.5, new Rotation2d());
        public static final Pose2d SUPERCYCLE_TARGET_POSE_RED = new Pose2d(Field.FIELD_LENGTH - 1.5,Field.FIELD_WIDTH-1.5, new Rotation2d());

        //Drive to pose for amping constants
        public static final double KP_TRANSLATION = 3; //For drive to pose
        public static final double KP_ROTATION = 0.06; //Drive to pose
        public static final double TRANSLATION_MARGIN = 0.05; //Drive to pose
        public static final double ROTATION_MARGIN = 1;

        //Drive while shooting constants
        public static final double AUTO_AIM_MAX_DRIVETRAIN_VEL = 6.4; //Max velocity of the drivetrain during auto aiming to speaker
        //Factor which is used to compensate the robot rotation when shooting while driving
        public static final double ROTATION_VELOCITY_COMPENSATION_FACTOR = 0.1;
        //Factor to use for the outtake to compensate the pivot angle when shooting while driving
        public static final double PIVOT_VELOCITY_COMPENSATION_FACTOR = 0.23;

        //Turn to pose constants
        public static final double AUTO_AIM_MARGIN = 1.5;
        public static final double AUTO_AIM_SETPOINT_MARGIN = 5;// 12;//7
        public static final double AUTO_AIM_MAX_VEL_SETPOINT = 1.5; //The drivetrain should be turning slower than this to be on target
        public static final double AUTO_AIM_MAX_ROT_VEL = 4.4;
        public static final double AUTO_AIM_KP = 0.1;// 0.17;
        public static final double AUTO_AIM_KD = 0;// 0.03;

        public static final double SUBWOOFER_WIDTH = 0.920; // ONSHAPE MODEL, distance from x=0 to the front of the subwoofer
//        public static final double SUBWOOFER_WIDTH = 0.900; // 1477 MODEL, distance from x=0 to the front of the subwoofer
//        public static final double SUBWOOFER_WIDTH = 0.95; // HIGHTIDE FIELD, distance from april tag wall to front of the subwoofer
    }

    /* ---------------------------------------- */
    /* CONTROLLERS */
    /* ---------------------------------------- */
    public static final double ORANGE_LEFTSTICK_DEADBAND = 0.03;
    public static final double ORANGE_RIGHTSTICK_DEADBAND = 0.03;
    public static final double XBOX_CONTROLLER_DEADBAND = 0.06;
    public static final double PLAYSTATION_CLIMBER_DEADBAND = 0.05;
    public static final double DRIVE_EXPONENT = 2.5;

    public static class LEDStrip{
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 45;
        public static final int LED_OFFSET = 0;
        public static final double ENABLED_PATTERN_DURATION = 0.25;
        public static final double DISABLED_PATTERN_DURATION = 0.3;
        public static final LEDPattern DISABLED_PATTERN = new SolidColorPattern();

        }

}
