package frc.team4481.lib.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Class representing the Limelight on the robot.
 * This class can be used to retrieve the latest pose estimates and latencies from the Limelight.
 */
public class Limelight {

    /**
     * Array storing the pose readings from the Limelight.
     * A pose reading contains the actual pose, but also other parameters such as the latency.
     * The (0,0) coordinate of the pose is on the left bottom of the field.
     * This is the coordinate system we use.
     */
    private double[] poseReading;

    /**
     * Array storing the pose reading from the Limelight.
     * Each pose reading contains the actual pose, but also other parameters such as the latency
     * The pose is measured with the target at (0,0)
     * This reading will only be used for some checks
     */
    private double[] poseReadingTargetSpace;

    //The area of the selected April Tag that the different Limelight see
    private double tagArea;

    //The time at which the Limelight determined the pose
    private double readingTime;

    //Estimate of the robot pose by the Limelight stored in a {@code Pose2d}
    private Pose2d poseEstimate;

    //Name of the limelight in the Networktables
    String name;

    //Threshold variables to check whether the estimated position is within bounds
    private double lowerXThreshold;
    private double upperXThreshold;
    private double lowerYThreshold;
    private double upperYThreshold;

    /**
     * Construct a new Limelight object.
     * This objects retrieves the estimated position from the Networktables and checks if the positions are valid
     * @param name Name of the Limelight in the Networktables
     */
    public Limelight(String name){
        this.name = name;
    }

    /**
     * Set the boundary box within which the Limelight should accept the pose estimate.
     * The box is defined by the coordinates of the bottom left and top right corner.
     * The coordinate (0,0) is at the bottom left of the field when the blue alliance is at the left.
     * In the same view, the X coordinate increases when moving right
     * and the Y coordinate increases when moving up.
     */
    public void setBoundingBox(Translation2d bottomLeftCorner, Translation2d topRightCorner){
        lowerXThreshold = bottomLeftCorner.getX();
        lowerYThreshold = bottomLeftCorner.getY();
        upperXThreshold = topRightCorner.getX();
        upperYThreshold = topRightCorner.getY();
    }

    /**
     * Let the Limelight read the latest values from the Networktables
     */
    public void update(){
        poseReading = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
        poseReadingTargetSpace = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_targetspace").getDoubleArray(new double[]{0});
        tagArea = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(-1);
    }

    /**
     * Retrieve the latest pose estimate of the Limelight.
     * Before this is done, the {@code update()} function should always be called.
     * @return {@code Pose2d} storing the pose estimate, this is {@code null} if the estimate is not valid
     */
    public Pose2d getPoseEstimate(){
        //Check if the reading is empty based on the target space reading.
        //The target space reading is used, because the wpilib-blue readings (limelightPoseReadings) always contain some values,
        //even if the limelight should technically not see it.
        //The wpilib-bue reading will be used to checked if the position is valid.
        if (isPoseReadingEmpty() || isPoseReadingInValid()){
            return null;
        }
        return poseReadingToPose2d();
    }


    /**
     * Retrieve the time (in seconds) that the position was determined by the limelight
     * based on the latency that is stored in the reading array
     * @param currentTime the current time in seconds
     * @return the time when the pose was determined by the limelight
     */
    public double getReadingTime(double currentTime){
        double latency = poseReading[6] / 1000;
        return currentTime - latency;
    }

    /**
     * Retrieve the area of the most prioritized April Tag that the limelight sees
     * @return The relative area of the tag, this is -1 if it is not valid
     */
    public double getTagArea(){
        return tagArea;
    }

    /**
     * Check if a pose reading is empty (null or containing all zeros)
     * @return Whether the pose reading is empty
     */
    private boolean isPoseReadingEmpty(){
        //The pose reading in target space is used for this, because the limelight sets this reading to all zeros
        //if an april tag is too small to too close to an edge etc.
        if (poseReadingTargetSpace == null){
            return true;
        }
        //Loop through the poseReading and keep track of the sum
        //If the sum is not equal to zero during any step of the loop, the array is not empty
        double sum = 0;
        for (double value : poseReadingTargetSpace){
            sum += value;
            if (sum != 0){
                return false;
            }
        }
        return true;
    }

    /**
     * Check if the pose reading of the Limelight is valid,
     * i.e. the pose falls within limits
     * @return whether the pose is valid
     */
    private boolean isPoseReadingInValid(){
        //Check if the pose falls outside the limits of the field
        return (poseReading.length <= 5
                || poseReading[0] < lowerXThreshold
                || poseReading[0] > upperXThreshold
                || poseReading[1] < lowerYThreshold
                || poseReading[1] > upperYThreshold
                || poseReading[2] < 0); //Z
    }

    /**
     * Convert the double array to an actual {@code Pose2d}
     * @return {@code Pose2d} of the robot pose estimate on the field
     */
    private Pose2d poseReadingToPose2d(){
        return new Pose2d(new Translation2d(poseReading[0], poseReading[1]), Rotation2d.fromDegrees(poseReading[5]));
    }

}
