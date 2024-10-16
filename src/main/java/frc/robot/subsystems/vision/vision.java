// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;

public class vision extends SubsystemBase {
  PhotonCamera tagsCameraCorner, tagsCameraShooter, noteCamera;
  PhotonPipelineResult resultCorner, resultShooter, resultNoteCam;

  PhotonPoseEstimator poseEstimator1, poseEstimator2;
  Optional<EstimatedRobotPose> estimatedPose;

  /** Creates a new vision. */
  public vision() {
    tagsCameraCorner = new PhotonCamera("ArducamCorner");
    tagsCameraShooter = new PhotonCamera("ArducamShooter");
    noteCamera = new PhotonCamera("RazerCamNote");

    AprilTagFieldLayout fieldLayout;

    try {
      Transform3d cam = new Transform3d(new Translation3d(-0.32, -0.20, 0.58),
          new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));
      fieldLayout = new AprilTagFieldLayout(
          "/home/lvuser/deploy/vision/CustomAprilTagFieldLayout.json");
      poseEstimator1 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          tagsCameraCorner, cam);
      poseEstimator2 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          tagsCameraShooter, cam);
    } catch (IOException e) {
      e.printStackTrace();
    }
    resultNoteCam = noteCamera.getLatestResult();
    resultShooter = tagsCameraShooter.getLatestResult();
    resultCorner = tagsCameraCorner.getLatestResult();
  }

  public boolean hasNoteTargets() {
    return resultNoteCam.hasTargets();
  }

  public double getYaw() {
    return hasNoteTargets() ? resultNoteCam.getBestTarget().getYaw() -
        HighAltitudeConstants.NOTE_DETECTION_YAW_OFFSET : 0;
  }

  public double getPitch() {
    return hasNoteTargets() ? resultNoteCam.getBestTarget().getPitch() : 0;
  }

  public double getArea() {
    return hasNoteTargets() ? resultNoteCam.getBestTarget().getArea() : 0;
  }

  public List<PhotonTrackedTarget> getNoteTargets() {
    return resultNoteCam.getTargets();
  }

  public PhotonTrackedTarget getBiggestNoteTarget() {

    if (!hasNoteTargets())
      return null;

    PhotonTrackedTarget biggest = getNoteTargets().get(0);

    for (PhotonTrackedTarget target : getNoteTargets()) {

      if (target.getArea() > biggest.getArea())
        biggest = target;

    }
    return biggest;

  }

  public Optional<EstimatedRobotPose> getEstimatedPosition() {

    return estimatedPose;

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

    poseEstimator1.setReferencePose(prevEstimatedRobotPose);
    poseEstimator2.setReferencePose(prevEstimatedRobotPose);
    return estimatedPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
