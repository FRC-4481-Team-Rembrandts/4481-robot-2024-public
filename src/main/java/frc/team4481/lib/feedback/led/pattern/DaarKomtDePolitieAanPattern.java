package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;
import frc.team4481.lib.util.CountingDelay;

public class DaarKomtDePolitieAanPattern implements LEDPattern{

    private Color.RGB redColor = new Color.RGB(255,0,0);
    private Color.RGB blueColor = new Color.RGB(0,0,255);
    private final CountingDelay blinkDelay = new CountingDelay();
    private final CountingDelay solidDelay = new CountingDelay();
    private int microsecondFlip = 300000;

    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip  The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int offset = strip.getOffset();
        int length = strip.getLength();
        int halfLength = length / 2;

        Color.RGB firstColor = redColor;
        Color.RGB secondColor = blueColor;

        // Switching states should be done twice per duration
        if (solidDelay.delay(3)) {
            // After two seconds stop blinking
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                firstColor = redColor;
                secondColor = redColor;
            } else {
                firstColor = blueColor;
                secondColor = blueColor;
            }
        }else if (RobotController.getFPGATime() % microsecondFlip*2 < microsecondFlip) {
            // If two seconds have not passed
            firstColor = redColor;
            secondColor = blueColor;
        } else {
            firstColor = blueColor;
            secondColor = redColor;
        }

        // Assign color to first half of ledstrip
        for (int i = 0; i < halfLength; i++) {
            buffer.setRGB(
                    offset + i,
                    firstColor.red(),
                    firstColor.green(),
                    firstColor.blue()
            );
        }

        // Assign color to second half of ledstrip
        for (int i = halfLength; i < length; i++) {
            buffer.setRGB(
                    offset + i,
                    secondColor.red(),
                    secondColor.green(),
                    secondColor.blue()
            );
        }
    }
    @Override
    public String getName() {
        return "DaarKomtDePolitieAanPattern";
    }
}
