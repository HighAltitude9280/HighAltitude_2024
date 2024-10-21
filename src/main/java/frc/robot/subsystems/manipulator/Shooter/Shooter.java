// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Shooter extends SubsystemBase {
  HighAltitudeMotorGroup shooterLeftMotors;
  HighAltitudeMotorGroup shooterRightMotors;
  private boolean rpmOnTarget = false;

  private AnalogInput proximitySensor;

  private PIDController leftPidController;
  private PIDController rightPidController;
  private SimpleMotorFeedforward leftFeedforward;
  private SimpleMotorFeedforward rightFeedforward;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterLeftMotors = new HighAltitudeMotorGroup(RobotMap.SHOOTER_LEFT_MOTOR_PORTS,
        RobotMap.SHOOTER_LEFT_INVERTED_MOTORS_PORTS,
        RobotMap.SHOOTER_LEFT_MOTOR_TYPES);

    shooterRightMotors = new HighAltitudeMotorGroup(RobotMap.SHOOTER_RIGHT_MOTOR_PORTS,
        RobotMap.SHOOTER_RIGHT_INVERTED_MOTORS_PORTS,
        RobotMap.SHOOTER_RIGHT_MOTOR_TYPES);


    shooterLeftMotors.setEncoderInverted(RobotMap.SHOOTER_LEFT_ENCODER_IS_INVERTED);
    shooterRightMotors.setEncoderInverted(RobotMap.SHOOTER_RIGHT_ENCODER_IS_INVERTED);

    shooterLeftMotors.setBrakeMode(HighAltitudeConstants.SHOOTER_MOTORS_BRAKING_MODE);
    shooterRightMotors.setBrakeMode(HighAltitudeConstants.SHOOTER_MOTORS_BRAKING_MODE);
    proximitySensor = new AnalogInput(RobotMap.SHOOTER_PROXIMITY_SENSOR_PORT);

    leftFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SHOOTER_LEFT_kS,
        HighAltitudeConstants.SHOOTER_LEFT_kV);

    leftPidController = new PIDController(HighAltitudeConstants.SHOOTER_LEFT_kP,
        0, HighAltitudeConstants.SHOOTER_LEFT_kD);

    rightFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SHOOTER_RIGHT_kS,
        HighAltitudeConstants.SHOOTER_RIGHT_kV);

    rightPidController = new PIDController(HighAltitudeConstants.SHOOTER_RIGHT_kP,
        0, HighAltitudeConstants.SHOOTER_RIGHT_kD);
  }

  public void driveShooter(double speed) {
    shooterLeftMotors.setAll(speed);
    shooterRightMotors.setAll(speed);
  }

  public void stopShooter() {
    shooterLeftMotors.setAll(0);
    shooterRightMotors.setAll(0);
  }

  public void driveLeft(double speed) {
    shooterLeftMotors.setAll(speed);
  }

  public void stopLeft() {
    shooterLeftMotors.setAll(0);
  }

  public void driveRight(double speed) {
    shooterRightMotors.setAll(speed);
  }

  public void stopRight() {
    shooterRightMotors.setAll(0);
  }

  public double getShooterLeftVel() {
    return shooterLeftMotors.getEncoderVelocity();
  }

  public double getShooterRightVel() {
    return shooterRightMotors.getEncoderVelocity();
  }

  public boolean controlShooter(int rpm) {
    double leftOutput = leftFeedforward.calculate(rpm);
    leftOutput += leftPidController.calculate(getShooterLeftVel(), rpm);
    driveLeft(leftOutput);

    double rightOutput = rightFeedforward.calculate(rpm);
    rightOutput += rightPidController.calculate(getShooterRightVel(), rpm);
    driveRight(rightOutput);

    double deltaLeft = rpm - getShooterLeftVel();
    double deltaRight = rpm - getShooterRightVel();

    rpmOnTarget = (Math.abs(deltaRight) <= HighAltitudeConstants.SHOOTER_ON_TARGET
        && Math.abs(deltaLeft) <= HighAltitudeConstants.SHOOTER_ON_TARGET);

    SmartDashboard.putBoolean("Shooter onTarget", Math.abs(deltaLeft) <= HighAltitudeConstants.SHOOTER_ON_TARGET);
    return rpmOnTarget;
  }

  public boolean driveRPMToSpeaker() {
    return controlShooter(distanceToRPM(Robot.getRobotContainer().getSwerveDriveTrain().distanceToSpeaker()));
  }

  public void shootToSpeakerAutoIndexer() {
    if (driveRPMToSpeaker())
      Robot.getRobotContainer().getIndexer().indexerOut();
  }

  public void setRPMPower(double power) {
    currentRPMPowerLeft = power;
    currentRPMPowerRight = power;
  }

  public boolean hasNote() {
    return proximitySensor.getAverageValue() > 1000;
  }

  public boolean onRPMTarget() {
    return rpmOnTarget;
  }

  /**
   * Converts distance (in meters) to ideal RPM shooting power. This should be
   * mapped at each event with the actual field.
   * 
   * @param distance
   * @return
   */
  public static int distanceToRPM(double distance) {
    return 3000;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Left Velocity", getShooterLeftVel());
    SmartDashboard.putNumber("Shooter Bottom Velocity", getShooterRightVel());
    SmartDashboard.putNumber("Proximity", proximitySensor.getAverageValue());

    SmartDashboard.putBoolean("ShooterHasNote", hasNote());

  }
}
