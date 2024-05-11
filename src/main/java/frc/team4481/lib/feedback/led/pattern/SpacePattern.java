package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.feedback.led.LEDStrip;
import java.util.Random;
import java.util.stream.IntStream;
import java.util.Arrays;

public class SpacePattern implements LEDPattern{

    private int seed = 4481;
    private int maxk = 2;
    private int n = 500;
    private Random rng = new Random(seed);
    private int[] numbers = IntStream.rangeClosed(1, maxk).toArray();
    private double[][] amplitudesList = new double[maxk][];
    private double[][] shiftsList = new double[maxk][];

    public SpacePattern (){
        for (int i = 0; i < maxk; i++) {
            amplitudesList[i] = periodicFuncRand(rng, 4, 0.3, n);
            shiftsList[i] = periodicFuncRand(rng, 4, Math.PI / 4, n);
        }
    }
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {

        // Update with 25 fps
        double time = Timer.getFPGATimestamp();
        int t = ((int) Math.round(time * 50)) % n;

        int ledLength = strip.getLength();

        double[] hue = IntStream.range(0, ledLength).mapToDouble(i ->
                periodicFunc(numbers,
                        IntStream.range(0, maxk).mapToDouble(j -> amplitudesList[j][t]).toArray(),
                        IntStream.range(0, maxk).mapToDouble(k -> shiftsList[k][t]).toArray(),
                        i / (double) ledLength)
        ).toArray();

        for (int i = 0; i < ledLength; i++) {
            buffer.setHSV(i, ((int) (hue[i] * 180)) % 180, 255, 255);
        }
    }


    @Override
    public String getName() {
        return "SpacePattern";
    }

    private static double periodicFunc(int[] numbers, double[] amplitudes, double[] shifts, double t) {
        double sum = 0;
        for (int i = 0; i < numbers.length; i++) {
            sum += amplitudes[i] * Math.cos(numbers[i] * t * 2 * Math.PI + shifts[i]);
        }
        return sum;
    }

    private static double[] periodicFuncRand(Random rng, int maxk, double scale, int n) {
        int[] numbers = IntStream.rangeClosed(1, maxk).toArray();
//        double[] amplitudes = rng.doubles(maxk, -scale, scale).toArray();
        double[] amplitudes = rng.doubles(maxk, -scale, scale).sorted().map(d -> -d).toArray();
        double[] shifts = rng.doubles(maxk, -Math.PI, Math.PI).toArray();
        return IntStream.range(0, n).mapToDouble(i -> periodicFunc(numbers, amplitudes, shifts, i / (double) n)).toArray();
    }
}
