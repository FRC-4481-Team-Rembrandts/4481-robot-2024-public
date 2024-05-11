/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.team4481.lib.hardware;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class LimelightPoseEstimator {

    private AprilTagFieldLayout fieldTags;
    private PhotonCamera camera;
    private Transform3d robotToCamera;
    private Transform3d invRobotToCamera;
    private double poseCacheTimestampSeconds = -1;
    private final double ambiguityThreshold;
    private final double tagAreaThreshold;
    private StructPublisher<Pose2d> pose2dPublisher;

    private StructPublisher<Pose3d> pose3dPublisher;



    public LimelightPoseEstimator(
            AprilTagFieldLayout fieldTags,
            PhotonCamera camera,
            Transform3d robotToCamera,
            double ambiguityThreshold,
            double tagAreaThreshold) {
        this.fieldTags = fieldTags;
        this.camera = camera;
        this.robotToCamera = robotToCamera;
        this.invRobotToCamera = robotToCamera.inverse();
        this.ambiguityThreshold = ambiguityThreshold;
        this.tagAreaThreshold = tagAreaThreshold;

        pose2dPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/"+camera.getName()+"/estimated pose2d", Pose2d.struct).publish();
        pose3dPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/"+camera.getName()+"/pose3d of largest tag", Pose3d.struct).publish();
    }

    public Optional<EstimatedPose3d> getEstimatedPose(Pose2d currentEstimatedPose) {

        if (camera == null ){
            return Optional.empty();
        }
        if (!camera.isConnected()){
            return Optional.empty();
        }

        PhotonPipelineResult cameraResult = camera.getLatestResult();;


        if (cameraResult == null){
            return Optional.empty();
        }

        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
            // STOP COMPUTING
        }

        poseCacheTimestampSeconds = cameraResult.getTimestampSeconds();

        if (!cameraResult.hasTargets()) {
            return Optional.empty();
        }

        // Get number of tags
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/number of tags", cameraResult.targets.size());

        // Get target with largest area
        PhotonTrackedTarget target = cameraResult.targets.get(0);

        // if all targets are minimally the area threshold
        boolean allLargeTagArea = target.getArea() > tagAreaThreshold;
        for (int i = 1; i < cameraResult.targets.size(); i++) {
            PhotonTrackedTarget otherTarget = cameraResult.targets.get(i);
            if (target.getArea() < otherTarget.getArea()) {
                target = otherTarget;
            }
            allLargeTagArea = allLargeTagArea && target.getArea() > tagAreaThreshold;
        }
        SmartDashboard.putBoolean("Vision/"+camera.getName()+"/all tags have area above threshold", allLargeTagArea);

        // Publish position of largest tag (for debug only)
//        pose3dPublisher.set((new Pose3d(currentEstimatedPose)).transformBy(robotToCamera).transformBy(target.getBestCameraToTarget()));

        // Use multitarget
        if (cameraResult.getMultiTagResult().estimatedPose.isPresent && allLargeTagArea) {
            // Helemaal weekend
            return Optional.of(
                    new EstimatedPose3d(
                            new Pose3d()
                                    .plus(cameraResult.getMultiTagResult().estimatedPose.best) // field-to-camera
                                    .relativeTo(fieldTags.getOrigin())
                                    .plus(invRobotToCamera), // field-to-robot
                            cameraResult.getTimestampSeconds()
                    )
            );
        }

        // Use single target with largest area
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/singleTarget/tagArea",target.getArea());
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/singleTarget/tagAmbiguity",target.getPoseAmbiguity());
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/singleTarget/ID",target.getFiducialId());
        if (target.getPoseAmbiguity() < ambiguityThreshold && target.getArea() > tagAreaThreshold) {
            int targetFiducialId = target.getFiducialId();
            Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

            // Return empty if tag ID is unknown
            if (targetPosition.isEmpty()) {
                DriverStation.reportError(
                        "Tried to get pose of unknown AprilTag: " + targetFiducialId, false);
                return Optional.empty();
            }
            return Optional.of(
                new EstimatedPose3d(
                    targetPosition
                        .get()
                        .transformBy(target.getBestCameraToTarget().inverse())
                        .transformBy(invRobotToCamera),
                    cameraResult.getTimestampSeconds()
                )
            );
        }

        // Return empty if no tags are good enough
        return Optional.empty();
    }

    public Optional<EstimatedPose2d> getFilteredEstimatedPose(Pose2d currentEstimatedPose) {
        // Check field dimensions
        Optional<EstimatedPose3d> estimatedPose = getEstimatedPose(currentEstimatedPose);

        // If empty still empty
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        // If pose is NOT in field return empty
        if (!isPoseInField(estimatedPose.get().estimatedPose)) {
            return Optional.empty();
        }

        // Publish estimated pose2d
        pose2dPublisher.set(estimatedPose.get().estimatedPose.toPose2d());

        return Optional.of(EstimatedPose2d.fromEstimatedPose3d(estimatedPose.get()));
    }

    private boolean isPoseInField(Pose3d pose){
        //Check if the pose falls outside the limits of the field
        Translation3d translation = pose.getTranslation();

        // TODO robot cannot intersect with field elements

        return (translation.getX() > 0
                && translation.getX() < fieldTags.getFieldLength()
                && translation.getY() > 0
                && translation.getY() < fieldTags.getFieldWidth()
                && translation.getZ() < 0.4); // Height lower than 0.4
    }

    public static class EstimatedPose3d {
        public final Pose3d estimatedPose;
        public final double timestampSeconds;

        public EstimatedPose3d(
                Pose3d estimatedPose,
                double timestampSeconds) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
        }
    }

    public static class EstimatedPose2d {
        public final Pose2d estimatedPose;
        public final double timestampSeconds;

        public EstimatedPose2d(
                Pose2d estimatedPose,
                double timestampSeconds) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
        }

        public static EstimatedPose2d fromEstimatedPose3d(EstimatedPose3d estimatedPose) {
            return new EstimatedPose2d(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds
            );
        }
    }
}
