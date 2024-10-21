// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Indexer extends SubsystemBase {
  HighAltitudeMotorGroup indexerMotors;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotors = new HighAltitudeMotorGroup(RobotMap.SHOOTER_INDEXER_MOTOR_PORTS,
        RobotMap.SHOOTER_INDEXER_INVERTED_MOTORS_PORTS,
        RobotMap.SHOOTER_INDEXER_MOTOR_TYPES);
  }

  public void driveIndexer(double speed) {
    indexerMotors.setAll(speed);
  }

  public void indexerOut() {
    indexerMotors.setAll(0.7);
  }

  // Drives the rollers out if the shooter is on its RPM target.
  public void autoRollersOutRPMTarget() {
    if (Robot.getRobotContainer().getShooter().onRPMTarget())
      indexerOut();
  }

  public void stopRollers() {
    indexerMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
