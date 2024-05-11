package frc.team4481.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.feedback.led.*;
import frc.team4481.lib.feedback.led.pattern.*;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.Constants;
import frc.team4481.robot.util.ScoringHandler;

import static frc.team4481.robot.Constants.LEDStrip.*;

public class Utility extends SubsystemBase<UtilityManager> {
    private final SubsystemHandler mSubsystemHandler = SubsystemHandler.getInstance();

    private final ScoringHandler scoringhandler = ScoringHandler.getInstance();
    private final PWMLEDController ledController = new PWMLEDController(LED_PORT);
    private LEDStrip strip = new LEDStrip(LED_LENGTH,LED_OFFSET);

    private boolean hadNote;


    public Utility (){
        name = "Utility";
        subsystemManager = new UtilityManager();
        try {
            strip.setPattern(DISABLED_PATTERN);
            strip.setPrimaryColor(new Color(0,0,0));
            strip.setPatternDuration(DISABLED_PATTERN_DURATION);
            ledController.addStrip(strip);
            ledController.start();
        } catch (DuplicateLEDAssignmentException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(UtilityManager.controlState.DISABLED);
        }else{
            subsystemManager.setControlState(UtilityManager.controlState.ENABLED);
            strip.setPatternDuration(ENABLED_PATTERN_DURATION);
            strip.setPattern(new BlinkPattern());
        }
        zeroSensors();
    }

    @Override
    public void readPeriodicInputs() {


    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getControlState()) {
            case DISABLED:


                break;
            case ENABLED:
                if (subsystemManager.isNoteInStorage()) {
                    //set colour to green
                    // No blink
                    strip.setPatternDuration(-1);
                    // green
                    strip.setSecondaryColor(Color.fromRGB(0, 255, 0));
                } else {
                    //quick blink
                    strip.setPatternDuration(ENABLED_PATTERN_DURATION);
                    // Turn off green
                    strip.setSecondaryColor(Color.fromRGB(255, 255, 255));
                }

                switch (scoringhandler.getScoringPosition()) {
                    case AMP:
                        //pink
                        strip.setPrimaryColor(Color.fromRGB(150,0,150));
                        break;
                    case SPEAKER:
                        //orange
                        strip.setPrimaryColor(Color.fromRGB(255,0,150));
                        break;
                    case LOPJE:
                        //blue
                        strip.setPrimaryColor(Color.fromRGB(0,0,255));
                }
                break;
        }
    }
    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void onStop(double timestamp) {
        strip.setPattern(DISABLED_PATTERN);
        strip.setPatternDuration(DISABLED_PATTERN_DURATION);
        terminate();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void terminate() {
        subsystemManager.setControlState(UtilityManager.controlState.DISABLED);
    }

    @Override
    public void outputData() {
        ledController.updateStrips();
        SmartDashboard.putString("Utility/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putBoolean("Utility/storage occupied", subsystemManager.isNoteInStorage());
        SmartDashboard.putString("Utility/LED/strip parttern", strip.getPattern().getName());
        SmartDashboard.putNumber("Utility/LED/strip primary color", strip.getPrimaryColor().getHSV().hue());
        SmartDashboard.putNumber("Utility/LED/strip secondary color", strip.getSecondaryColor().getHSV().hue());
    }

}

