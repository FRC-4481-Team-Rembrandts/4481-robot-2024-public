package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;

import java.util.ArrayList;
import java.util.Random;

/**
 * A pattern that can be applied to an LED strip.
 * This pattern will mimic the selection sort algorithm.
 * It will override the hue of the primary color.
 * The saturation and value of the primary color remain the same.
 * The secondary color is not used.
 * The pattern duration is approximated but might not be exact.
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public class SelectionSortPattern implements LEDPattern {
    private int[] sortArray = new int[0];
    private final ArrayList<int[]> sortSteps = new ArrayList<>();

    private double stepShiftBuffer = 0;
    private double previousTime = Timer.getFPGATimestamp();
    private int stepIndex = 0;

    private boolean sorted = false;
    private boolean shuffled = false;

    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip  The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        final int HUE_RANGE = 180;

        int offset = strip.getOffset();
        int length = strip.getLength();
        Color.HSV color = strip.getPrimaryColor().getHSV();
        double patternSeconds = strip.getPatternDuration();

        // Calculate delta time
        double newTime = Timer.getFPGATimestamp();
        double deltaTime = newTime - previousTime;
        previousTime = newTime;

        // Create new array if needed
        if (sortArray.length != length) {
            sortArray = new int[length];

            for (int i = 0; i < length; i++) {
                sortArray[i] = i * HUE_RANGE / length % HUE_RANGE;
            }

            shuffled = false;
        }

        // Shuffle array if needed
        if (!shuffled) {
            sortSteps.clear();

            shuffleArray(sortArray);

            sortSteps.add(sortArray.clone());

            shuffled = true;
            sorted = false;
            stepIndex = 0;
        }

        // Sort array if needed
        if (!sorted) {
            selectionSort(sortArray, sortSteps);
            sorted = true;
        }

        // Calculate delta time
        stepShiftBuffer += ((deltaTime / patternSeconds) * sortSteps.size());
        int stepShift = (int) stepShiftBuffer;
        stepShiftBuffer -= stepShift;
        stepIndex += stepShift;


        // Check bounds
        if (stepIndex >= sortSteps.size()) {
            shuffled = false;
        } else {
            int[] step = sortSteps.get(stepIndex);

            // Set colors
            for (int i = 0; i < length; i++) {
                int hue = step[i];
                buffer.setHSV(i + offset, hue, color.saturation(), color.value());
            }
        }
    }

    /**
     * Sorts an array using the selection sort algorithm.
     * @param arr The array to be sorted.
     * @param sortSteps The list to add the steps to.
     */
    public static void selectionSort(int[] arr, ArrayList<int[]> sortSteps){
        for (int i = 0; i < arr.length - 1; i++)
        {
            int index = i;
            for (int j = i + 1; j < arr.length; j++){
                if (arr[j] < arr[index]){
                    index = j;//searching for lowest index
                }
            }
            int smallerNumber = arr[index];
            arr[index] = arr[i];
            arr[i] = smallerNumber;

            sortSteps.add(arr.clone());
        }
    }

    /**
     * Shuffles an array.
     * @param array The array to be shuffled.
     */
    private static void shuffleArray(int[] array) {
        Random rand = new Random();

        for (int i = 0; i < array.length; i++) {
            int randomIndexToSwap = rand.nextInt(array.length);
            int temp = array[randomIndexToSwap];
            array[randomIndexToSwap] = array[i];
            array[i] = temp;
        }
    }

    @Override
    public String getName() {
        return "SelectionSort";
    }
}
