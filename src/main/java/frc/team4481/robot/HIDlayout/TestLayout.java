package frc.team4481.robot.HIDlayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.controller.IPS4HID;
import frc.team4481.lib.filter.*;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.util.ScoringHandler;
import frc.team4481.robot.subsystems.Climber;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.Outtake;
import frc.team4481.robot.subsystems.OuttakeManager;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.autoaim.DriveToPose;
import frc.team4481.robot.autoaim.TurnToPose;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.subsystems.Drivetrain;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.team4481.robot.Constants.*;
import static frc.team4481.robot.Constants.AutoAim.*;
import static frc.team4481.robot.Constants.Drivetrain.*;
import static frc.team4481.robot.util.ScoringHandler.ScoringPosition.*;


public class TestLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private final ScoringHandler scoringHandler = ScoringHandler.getInstance();
    private Drivetrain drivetrain;
    private DrivetrainManager drivetrainManager;
    private Climber climber;
    private ClimberManager climberManager;
    private ExponentScaleFilter exponentScaleFilter;
    private Intake intake;
    private IntakeManager intakeManager;
    private Outtake outtake;
    private OuttakeManager outtakeManager;
    private Utility utility;
    private UtilityManager utilityManager;
    private DeadZoneFilter deadzoneFilter;
    private DeadZoneFilter deadzoneFilterClimber;
    private ScaleSpeedsFilter invertFilter;
    private DiscretizeFilter discretizeFilter;
    private HeadingCorrectionFilter headingCorrectionFilter;
    private AdaptiveSlewRateLimiter slewRateLimiter;
    private ScaleSpeedsFilter scaleSpeedsFilter;
    private UnfilteredChassisSpeeds unfilteredChassisSpeeds;
    private UnfilteredChassisSpeeds unfilteredClimberChassisSpeeds;
    private ChassisSpeedsFilter filteredChassisSpeeds;
    private DriveToPose drivetoPose;
    private TurnToPose turnToPose;
    private Pose2d targetSpeaker;
    private Pose2d targetAmp;
    private double climbingSpeed;
    public OuttakeManager.positionState operatorOverride = null;

    private LockRotationFilter lockRotationFilter;
    private CountingDelay ejectDelay = new CountingDelay();
    private CountingDelay intakeHoldDelay = new CountingDelay();
    private boolean disableDriveToPos = false;
    private boolean autoAmping = false;

    private CountingDelay endgameVibrationDelay = new CountingDelay();



    public TestLayout(XboxController driver, ControlDevice operator) {
        super(driver, operator);


        //Set the current alliance that the robot is on
//        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
//            alliance = DriverStation.Alliance.Red;
//        } else {
//            alliance = DriverStation.Alliance.Blue;
//        }

        // ChassisSpeeds filters
        // If these filters include matrix operations,
        // Always apply these in order: scaling > rotation > translation
        unfilteredChassisSpeeds = new UnfilteredChassisSpeeds();
        deadzoneFilter = new DeadZoneFilter(
                unfilteredChassisSpeeds,
                XBOX_CONTROLLER_DEADBAND
        );
        exponentScaleFilter = new ExponentScaleFilter(
                deadzoneFilter, DRIVE_EXPONENT
        );
        scaleSpeedsFilter = new ScaleSpeedsFilter(
                exponentScaleFilter
        );
        invertFilter = new ScaleSpeedsFilter(
                scaleSpeedsFilter
        );

        unfilteredClimberChassisSpeeds = new UnfilteredChassisSpeeds();
        deadzoneFilterClimber = new DeadZoneFilter(
                unfilteredClimberChassisSpeeds,
                PLAYSTATION_CLIMBER_DEADBAND
        );

        slewRateLimiter = new AdaptiveSlewRateLimiter(
                invertFilter,
                REVERSE_THRESHOLD
        );

        headingCorrectionFilter = new HeadingCorrectionFilter(
                slewRateLimiter,
                HEADING_CORRECTION_MARGIN,
                HEADING_CORRECTION_KP,
                HEADING_CORRECTION_TURNON_TIME
        );
        discretizeFilter = new DiscretizeFilter(
                headingCorrectionFilter,
                kHIDLooperDt
        );

        filteredChassisSpeeds = discretizeFilter;

        lockRotationFilter = new LockRotationFilter(
                filteredChassisSpeeds,
                2,
                0.05
        );


        //Initialize turn to pose object
        turnToPose = new TurnToPose(AUTO_AIM_MARGIN,AUTO_AIM_SETPOINT_MARGIN,AUTO_AIM_KP,AUTO_AIM_KD, ROTATION_VELOCITY_COMPENSATION_FACTOR);


        drivetoPose = new DriveToPose(AutoAim.KP_TRANSLATION, AutoAim.KP_ROTATION, AutoAim.TRANSLATION_MARGIN, AutoAim.ROTATION_MARGIN);

        endgameVibrationDelay.reset();
    }

    @Override
    public void getSubsystemManagers() {
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();

        climber = (Climber) subsystemHandler.getSubsystemByClass(Climber.class);
        climberManager = climber.getSubsystemManager();

        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakeManager = outtake.getSubsystemManager();

        utility = (Utility) subsystemHandler.getSubsystemByClass(Utility.class);
        utilityManager = utility.getSubsystemManager();
    }

    @Override
    public void updateOrange() throws HardwareException {
        //Update the config
        Configuration config = configHandler.getConfig();


        //Set target pose
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            targetAmp = AutoAim.AMP_SCORE_POSE_RED;
            targetSpeaker = AutoAim.SPEAKER_TARGET_POSE_RED;
        } else {
            targetAmp = AutoAim.AMP_SCORE_POSE_BLUE;
            targetSpeaker = AutoAim.SPEAKER_TARGET_POSE_BLUE;
        }


        //Drivetrain control
        ChassisSpeeds baseChassisSpeed = new ChassisSpeeds(
                -driver.getLeftY(),
                -driver.getLeftX(),
                -driver.getRightX()
        );

        unfilteredChassisSpeeds.setBaseSpeeds(baseChassisSpeed);

        //Update the angular scale filter, the translation scale is updated below
        scaleSpeedsFilter.setAngularScale(config.getMaxTurnVelocity());

        //If the alliance is red, we should use the invert filter for the next filter
        //otherwise, we can continue with the scale speed filter
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            invertFilter.setScale(-1.0);
        } else {
            invertFilter.setScale(1.0);
        }

        //Update the direction change limiter
        slewRateLimiter.setRateLimit(
                scaleDirectionRateLimit(baseChassisSpeed, MIN_DIRECTION_RATE_LIMIT)
        );

        //Update the heading correction filter
        headingCorrectionFilter.update(drivetrainManager.getPoseEstimate().getRotation(), kHIDLooperDt);

        //Determine the chassisspeed for the drivetrain
        ChassisSpeeds desiredSpeed = new ChassisSpeeds();

        //drivetrain autoAim (turnToPose)
        if ((operatorOverride == null || operatorOverride == OuttakeManager.positionState.OVERRIDE_AUTOMATIC)
                && (scoringHandler.getScoringPosition() == SPEAKER
                || scoringHandler.getScoringPosition() == LOPJE)) {
            //If no override is present and the score position is not amp, the drivetrain can automatically rotate to the speaker
            if (driver.getRightBumper() || driver.getRightTriggerAxis() > 0.1) {
                //Reduce the maximum speed that the driver can input by adjusting the scale filter
                scaleSpeedsFilter.setScale(AUTO_AIM_MAX_DRIVETRAIN_VEL);
                //Get the x and y speeds from the controller and the rotation from the look to pose function
                desiredSpeed.vxMetersPerSecond = filteredChassisSpeeds.getFilteredSpeeds().vxMetersPerSecond;
                desiredSpeed.vyMetersPerSecond = filteredChassisSpeeds.getFilteredSpeeds().vyMetersPerSecond ;
                //Give the target robot speed to the turn to pose object to compensate for velocity
                turnToPose.updateRobotSpeed(drivetrainManager.getCurrentSpeeds());
                //Retrieve the target rotation speed
                desiredSpeed.omegaRadiansPerSecond = turnToPose.getTargetSpeeds(drivetrainManager.getPoseEstimate(), targetSpeaker).omegaRadiansPerSecond;
            } else {
                //Update the scale filter
                scaleSpeedsFilter.setScale(config.getMaxDriveVelocity());
                //Get the desired speeds through all the filters
                desiredSpeed = filteredChassisSpeeds.getFilteredSpeeds();
            }
        }
        else if (operatorOverride == null && scoringHandler.getScoringPosition() == AMP) {
            lockRotationFilter.update(drivetrainManager.getPoseEstimate().getRotation(), Rotation2d.fromDegrees(-90), kHIDLooperDt);

            SmartDashboard.putBoolean("DT/auto_amping/autoAmp", autoAmping);
            SmartDashboard.putBoolean("DT/auto_amping/disable_amp", disableDriveToPos);
            //If no override is present and the score position is not amp, the drivetrain can automatically align to the amp
            if ((driver.getRightBumper() || driver.getRightTriggerAxis() > 0.1) && !disableDriveToPos) {
                autoAmping = true;
                //Update the scale filter
                scaleSpeedsFilter.setScale(config.getMaxDriveVelocity());
                //Get the desired speeds through all the filters
                drivetoPose.setRobotSpeed(drivetrainManager.getCurrentSpeeds());
                desiredSpeed = drivetoPose.getTargetSpeeds(drivetrainManager.getPoseEstimate(),targetAmp);
            }else {
                autoAmping = false;
                //Update the scale filter
                scaleSpeedsFilter.setScale(config.getMaxDriveVelocity());
                //Get the desired speeds through all the filters
                desiredSpeed = filteredChassisSpeeds.getFilteredSpeeds();
            }
        }
        else {
            //Update the scale filter
            scaleSpeedsFilter.setScale(config.getMaxDriveVelocity());
            //Get the desired speeds through all the filters
            desiredSpeed = filteredChassisSpeeds.getFilteredSpeeds();
        }
        drivetrainManager.setDesiredSpeeds(desiredSpeed);

        //Reset position or rotation of the drivetrain if necessary
        if (driver.getBackButton() && driver.getStartButton()) {
            drivetrainManager.setDisableLL(true);
            DataLogManager.log("TURNED OFF LIMELIGHTS");

        }else if (driver.getBackButton() || driver.getStartButton()){
            //Reset the rotation of the drivetrain if the options or share button is pressed
            resetDrivetrainRotation(drivetrainManager.getPoseEstimate());
        }
        
        //Intake controls
        if (operator.getButtonValue(IPS4HID.Button.SQUARE)) {
            intakeManager.setControlState(IntakeManager.controlState.SOURCE_INTAKE);
        } else if (driver.getLeftBumper()){
            if (intakeManager.getControlState() != IntakeManager.controlState.STORE
                    && intakeManager.getControlState() != IntakeManager.controlState.HOLD
                    && intakeManager.getControlState() != IntakeManager.controlState.SOURCE_INTAKE){
                intakeManager.setControlState(IntakeManager.controlState.INTAKE);
            }
        } else if (Math.abs(driver.getLeftTriggerAxis()) > 0.05) {
            if (intakeManager.getControlState() != IntakeManager.controlState.SOURCE_INTAKE){
                intakeManager.setControlState(IntakeManager.controlState.REVERSE_INTAKE);
            }
        } else if (driver.getRightBumper()){
                    if (outtakeManager.getMovingState() == OuttakeManager.movingState.ON_TARGET &&
                       (outtakeManager.getPositionState() != OuttakeManager.positionState.STOWED ||
                       (outtakeManager.getControlState() == OuttakeManager.controlState.AUTOMATIC &&
                       turnToPose.isInSetpointMargin()
                       )) )
                    {
                        intakeManager.setControlState(IntakeManager.controlState.INTAKE_FEEDER);
                    }
        } else if (operatorOverride == OuttakeManager.positionState.EJECT){
            intakeManager.setControlState(IntakeManager.controlState.INTAKE_FEEDER);
        }
        //Statement below was removed because it caused the feeding to stop while the note still needed to be shot
//        else if (Math.abs(driver.getRightTriggerAxis()) > 0.05) {
//          intakeManager.setControlState(IntakeManager.controlState.DISABLED);
//        }
        else if ( (outtakeManager.getPositionState() != OuttakeManager.positionState.STOWED
                || outtakeManager.getControlState() == OuttakeManager.controlState.AUTOMATIC)
                && intakeManager.getControlState() == IntakeManager.controlState.INTAKE_FEEDER) {
            //If the outtake is not stowed (or in automatic mode) and the intake is feeding, keep feeding
        } else if ((intakeManager.getControlState() != IntakeManager.controlState.INTAKE &&
                intakeManager.getControlState() != IntakeManager.controlState.HOLD &&
                intakeManager.getControlState() != IntakeManager.controlState.STORE &&
                intakeManager.getControlState() != IntakeManager.controlState.SOURCE_INTAKE)
                || (intakeManager.getControlState() == IntakeManager.controlState.SOURCE_INTAKE && operator.getButtonValue(IPS4HID.Button.CROSS))
        ){
            //When the intake is in intaking, holding or storing, don't allow it to disable
            intakeManager.setControlState(IntakeManager.controlState.DISABLED);
        }

        SmartDashboard.putBoolean("DT/turn to pose setpoint margin", turnToPose.isInSetpointMargin());

        // LED update
        utilityManager.setNoteInStorage(intakeManager.isStorageOccupied());

        // MOTOR OVERHEAT TUDUUU TUDUUUU
//        drivetrainManager.setBlinkMode(outtakeManager.getMotorOverheat());

        // sensor blink then add cooldown and reset on sensor off

        // blink for 0.2 seconds if sensor true
        if (intakeManager.isSensorHigh() && !intakeHoldDelay.delay(0.4)) {
            drivetrainManager.setBlinkMode(true);
        } else if (!intakeManager.isSensorHigh()) {
            drivetrainManager.setBlinkMode(false);
            intakeHoldDelay.reset();
        } else {
            drivetrainManager.setBlinkMode(false);
        }




        //Outtake controls
        if (scoringHandler.getScoringPosition() == SPEAKER || scoringHandler.getScoringPosition() == LOPJE){
            if (operatorOverride != null && operatorOverride != OuttakeManager.positionState.OVERRIDE_AUTOMATIC) {
                //If there is an override from the operator, go to that setpoint
                outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);
                outtakeManager.setPositionState(operatorOverride);
            } else if (Math.abs(driver.getRightTriggerAxis()) > 0.05 ||
                    driver.getRightBumper() || operatorOverride == OuttakeManager.positionState.OVERRIDE_AUTOMATIC){
                outtakeManager.setControlState(OuttakeManager.controlState.AUTOMATIC);
                outtakeManager.setPositionState(OuttakeManager.positionState.STOWED);
            } else{
                outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);
                outtakeManager.setPositionState(OuttakeManager.positionState.STOWED);
            }
        } else if (scoringHandler.getScoringPosition() == AMP){
           if (Math.abs(driver.getRightTriggerAxis()) > 0.05 ||
                    driver.getRightBumper()) {
                outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);
                outtakeManager.setPositionState(OuttakeManager.positionState.AMP);
            } else if (operatorOverride == OuttakeManager.positionState.EJECT){
               outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);
               outtakeManager.setPositionState(OuttakeManager.positionState.EJECT);
           } else {
                outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);
                outtakeManager.setPositionState(OuttakeManager.positionState.STOWED);
            }
        }


        //Give the desired speed of the robot to the outtake manager to compensate for shooting while driving
        outtakeManager.setRobotSpeed(drivetrainManager.getCurrentSpeeds());
        //Give the current position of the robot to the outtake to determine the outtake setpoint from the lookup table
        outtakeManager.setRobotPose(drivetrainManager.getPoseEstimate());


        //Auto align
//        if (driver.getButtonValue(LEFTSTICK_BUTTON)) {
//            //Automatically drive to point
//            if (alliance == DriverStation.Alliance.Red){
//                desiredSpeed = drivetoPose.getTargetSpeeds(drivetrainManager.getPoseEstimate(),AutoAim.AMP_SCORE_POSE_RED);
//            }else{
//                desiredSpeed = drivetoPose.getTargetSpeeds(drivetrainManager.getPoseEstimate(),AutoAim.AMP_SCORE_POSE_BLUE);
//            }
//        } else

        if (DriverStation.getMatchTime() < 27 && !endgameVibrationDelay.delay(1) && DriverStation.isTeleop()) {
            driver.setRumble(kBothRumble,1.0);
        } else {
            driver.setRumble(kBothRumble,0.0);
        }

    }


    @Override
    public void updateBlack() throws HardwareException {
        //Switch between speaker and amp mode
        if (operator.getRawButtonPressed(IPS4HID.Button.BUMPER_R1.id)){
            SmartDashboard.putString("scoringHandler", scoringHandler.getScoringPosition().toString());
            if (scoringHandler.getScoringPosition() == SPEAKER){
                scoringHandler.setScoringPosition(AMP);

            } else if (scoringHandler.getScoringPosition() == AMP){
                scoringHandler.setScoringPosition(SPEAKER);

            } else if (scoringHandler.getScoringPosition() == LOPJE){
                scoringHandler.setScoringPosition(SPEAKER);
            }
        }
        if (operator.getAxisValue(IPS4HID.Axis.TRIGGER_R2) > 0.5){
            SmartDashboard.putString("scoringHandler", scoringHandler.getScoringPosition().toString());
            scoringHandler.setScoringPosition(LOPJE);
        }

        //Outtake control
        if (scoringHandler.getScoringPosition() == SPEAKER){
            if (operator.getButtonValue(IPS4HID.Button.BUMPER_L1)){
                operatorOverride = OuttakeManager.positionState.EJECT;
            } else if (operatorOverride == OuttakeManager.positionState.EJECT){
                //If the button is not pressed, but the mode is still EJECT, reset
                operatorOverride = null;
            }
            if (operator.getButtonValue(IPS4HID.Button.CROSS)){
                operatorOverride = null;
            }
            if (operator.getButtonValue(IPS4HID.Button.TRIANGLE)){
                operatorOverride = OuttakeManager.positionState.AGAINST_SUBWOOFER;
            }
            if (operator.getButtonValue(IPS4HID.Button.CIRCLE)) {
                operatorOverride = OuttakeManager.positionState.PODIUM;
            }
            if (operator.getButtonValue(IPS4HID.Button.SQUARE)) {
                operatorOverride = OuttakeManager.positionState.INTAKING;
            }
            if (getAnyDpadValue(operator)){
                operatorOverride = OuttakeManager.positionState.OVERRIDE_AUTOMATIC;
            }
        }else {
            operatorOverride = null;
            //Default the override the null, unless the note should be ejected
            if (operator.getButtonValue(IPS4HID.Button.BUMPER_L1)){
                operatorOverride = OuttakeManager.positionState.EJECT;

            } else if (operatorOverride == OuttakeManager.positionState.EJECT){
                //If the button is not pressed, but the mode is still EJECT, reset
                operatorOverride = null;
            }
        }

        if (operator.getButtonValue(IPS4HID.Button.RIGHTSTICK_BUTTON)){
            disableDriveToPos = true;
        }


        //Climbing control
        ChassisSpeeds baseClimberChassisSpeeds = new ChassisSpeeds(
                0,
                operator.getAxisValue(IPS4HID.Axis.LEFTSTICK_Y),
                0
        );
        unfilteredClimberChassisSpeeds.setBaseSpeeds(baseClimberChassisSpeeds);
        climberManager.setClimbingSpeed(deadzoneFilterClimber.getFilteredSpeeds().vyMetersPerSecond);

    }

    /**
     * Reset the rotation of the drivetrain.
     * The robot should be pointed away from the driver when this is called.
     * Because we use a coordinate system focussed around the blue alliance, the rotation should be flipped 180
     * degrees when on the red side
     * @param currentPose Current position of the robot
     */
    private void resetDrivetrainRotation(Pose2d currentPose){
        Pose2d newPose;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            newPose = new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(180));
        } else {
            newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        }

        drivetrainManager.setPoseToReset(newPose);
    }

    /**
     * Scale the direction change limit based on the raw input from the controller
     * @param baseSpeed Chassisspeed based on the raw controller values
     * @param minDirectionChangeLimit The harshest limit that can be applied in RAD/s
     * @return The scaled limit
     */
    private double scaleDirectionRateLimit(ChassisSpeeds baseSpeed, double minDirectionChangeLimit){
        //The distance of the controller joystick to the middle, is 1 at max
        double baseV = Math.hypot(baseSpeed.vxMetersPerSecond, baseSpeed.vyMetersPerSecond);
        if(baseV < 1e-3){
            return 1000; //Return a very large limit, resulting in the drivetrain changing heading unlimited
        }
        double scale = 1 / baseV;
        return minDirectionChangeLimit * scale;
    }

    private boolean getAnyDpadValue(ControlDevice controller) {
        return controller.getDpadValue(IPS4HID.DpadButton.DPAD_N) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_NE) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_E) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_SE) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_S) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_SW) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_W) ||
                controller.getDpadValue(IPS4HID.DpadButton.DPAD_NW);
    }
}



