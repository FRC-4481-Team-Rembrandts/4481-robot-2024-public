package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 * A pattern that can be applied to an LED strip.
 * This pattern will mimic the merge sort algorithm.
 * It will override the hue of the primary color.
 * The saturation and value of the primary color remain the same.
 * The secondary color is not used.
 * The pattern duration is approximated but might not be exact.
 * @see LEDStrip
 */
public class MergeSortPattern implements LEDPattern {
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
            mergeSort(sortArray, 0, sortArray.length - 1);
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

    private void mergeSort(int[] array, int left, int right) {
        if (left < right) {
            int mid = left + (right - left) / 2;

            // Recursively sort the two halves
            mergeSort(array, left, mid);
            mergeSort(array, mid + 1, right);

            // Merge the sorted halves
            merge(array, left, mid, right);

            // Add the entire array after each merge
            sortSteps.add(array.clone());
        }
    }

    /**
     * Sorts an array using the merge sort algorithm.
     * @param array The array to be sorted.
     * @param left The left index of the array.
     * @param right The right index of the array.
     * @param mid The middle index of the array.
     */
    public void merge(int[] array, int left, int mid, int right) {
        int n1 = mid - left + 1;
        int n2 = right - mid;

        int[] leftArray = new int[n1];
        int[] rightArray = new int[n2];

        // Copy data to temporary arrays
        for (int i = 0; i < n1; ++i) {
            leftArray[i] = array[left + i];
        }
        for (int j = 0; j < n2; ++j) {
            rightArray[j] = array[mid + 1 + j];
        }

        // Merge the temporary arrays back into the original array
        int i = 0, j = 0, k = left;
        while (i < n1 && j < n2) {
            if (leftArray[i] <= rightArray[j]) {
                array[k++] = leftArray[i++];
            } else {
                array[k++] = rightArray[j++];
            }
            sortSteps.add(array.clone());
        }

        // Copy the remaining elements of leftArray[], if there are any
        while (i < n1) {
            array[k++] = leftArray[i++];
            sortSteps.add(array.clone());
        }

        // Copy the remaining elements of rightArray[], if there are any
        while (j < n2) {
            array[k++] = rightArray[j++];
            sortSteps.add(array.clone());
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
        return "MergeSort";
    }
}
