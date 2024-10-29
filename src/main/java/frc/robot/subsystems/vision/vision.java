// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
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

  PhotonPoseEstimator poseEstimatorCorner, poseEstimatorShooter;

  private ArrayList<Optional<EstimatedRobotPose>> estimatedPoseCorner, estimatedPoseShooter;

  private Optional<EstimatedRobotPose> pose1;
  private Optional<EstimatedRobotPose> pose2;

  /** Creates a new vision. */
  public vision() {
    tagsCameraCorner = new PhotonCamera("ArducamCorner");
    tagsCameraShooter = new PhotonCamera("ArducamShooter");
    noteCamera = new PhotonCamera("RazerCamNote");

    estimatedPoseCorner = new ArrayList<>();
    estimatedPoseShooter = new ArrayList<>();

    AprilTagFieldLayout fieldLayout;
    try {
      // Translation 3d use this doc to know the position on your robot.
      // https://docs.wpilib.org/es/stable/docs/software/basic-programming/coordinate-system.html
      fieldLayout = new AprilTagFieldLayout(
          "/home/lvuser/deploy/vision/CustomAprilTagFieldLayout.json");

      Transform3d camCorner = new Transform3d(new Translation3d(0, 0, 0),
          new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));

      poseEstimatorCorner = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          tagsCameraCorner, camCorner);
      Transform3d camShooter = new Transform3d(new Translation3d(0, 0, 0),
          new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));
      poseEstimatorShooter = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          tagsCameraShooter, camShooter);

    } catch (IOException e) {

      e.printStackTrace();
    }

    // Initialize pose1 and pose2 after poseEstimators are created
    pose1 = poseEstimatorShooter.update();
    pose2 = poseEstimatorCorner.update();

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

  public ArrayList<Optional<EstimatedRobotPose>> getEstimatedPosition() {
    ArrayList<Optional<EstimatedRobotPose>> result = new ArrayList<>();
    result.add(pose1);
    result.add(pose2);
    return result;
  }

  public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimatorShooter.setReferencePose(prevEstimatedRobotPose);
    ArrayList<Optional<EstimatedRobotPose>> result = new ArrayList<>();
    result.add(pose1);
    result.add(pose2);
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose1 = poseEstimatorShooter.update();
    pose2 = poseEstimatorCorner.update();
    getEstimatedPosition();
    getEstimatedGlobalPose(null);
  }
}
