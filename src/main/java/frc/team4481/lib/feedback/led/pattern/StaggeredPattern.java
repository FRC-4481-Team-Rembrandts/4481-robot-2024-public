package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;

public class StaggeredPattern implements LEDPattern {

    private int SEGMENT_LENGTH = 8;


    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int length = strip.getLength();
        Color.RGB primaryColor = strip.getPrimaryColor().getRGB();
        Color.RGB secondaryColor = strip.getSecondaryColor().getRGB();
        double blinkDuration = strip.getPatternDuration(); // Negative duration is solid color
        double microsecondFlip = blinkDuration * 10e6;

        Color.RGB firstColor;
        Color.RGB secondColor;

        if (RobotController.getFPGATime() % microsecondFlip*2 < microsecondFlip && blinkDuration >= 0) {
            firstColor = primaryColor;
            secondColor = secondaryColor;
        } else {
            firstColor = secondaryColor;
            secondColor = primaryColor;
        }

        for (int i = 0; i < length; i++) {
            if (i % SEGMENT_LENGTH * 2 < SEGMENT_LENGTH) {
                buffer.setRGB(
                        i,
                        firstColor.red(),
                        firstColor.green(),
                        firstColor.blue()
                );
            } else{
                buffer.setRGB(
                        i,
                        secondColor.red(),
                        secondColor.green(),
                        secondColor.blue()
                );
            }
        }
    }

    @Override
    public String getName() {
        return "StaggeredPattern";
    }
}
