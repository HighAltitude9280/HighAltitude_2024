// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;
import frc.robot.subsystems.vision.vision;

public class ShooterPivot extends SubsystemBase {
  HighAltitudeMotorGroup shooterPivotMotors;
  double ShooterPivotEncoderPosition, shooterPivotPositionDegrees, shooterPivotRawEncoder;
  double currentTarget;

  DigitalInput topLimitSwitch;
  DigitalInput bottomLimitSwitch;

  CANcoder absoluteEncoderController;

  double zeroValue = 0;

  boolean Override;

  double encoderTarget = 0.0, angleTarget = 65.0;

  SysIdRoutine routine;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {

    shooterPivotMotors = new HighAltitudeMotorGroup(RobotMap.SHOOTER_PIVOT_MOTOR_PORTS,
        RobotMap.SHOOTER_PIVOT_INVERTED_MOTORS_PORTS,
        RobotMap.SHOOTER_PIVOT_MOTOR_TYPES);

    shooterPivotMotors.setBrakeMode(HighAltitudeConstants.SHOOTER_PIVOT_MOTOR_BRAKING_MODE);
    shooterPivotMotors.setEncoderInverted(RobotMap.SHOOTER_PIVOT_ENCODER_IS_INVERTED);

    absoluteEncoderController = new CANcoder(RobotMap.SHOOTER_PIVOT_ENCODED_TALON_PORT);
    resetCanCoder();

    if (RobotMap.SHOOTER_PIVOT_TOP_LIMIT_SWITCH_IS_AVAILABLE) {
      topLimitSwitch = new DigitalInput(RobotMap.SHOOTER_PIVOT_TOP_LIMIT_SWITCH_PORT);
    }

    if (RobotMap.SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE) {
      bottomLimitSwitch = new DigitalInput(RobotMap.SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_PORT);
    }
    ;

    currentTarget = getShooterPivotPositionInDegrees();

    Override = false;

  }

  public double getAbsoluteEncoderDeg() {
    double angle = HighAltitudeConstants.SHOOTER_PIVOT_ZERO_ANGLE
        + (RobotMap.SHOOTER_PIVOT_ENCODED_TALON_INVERTED ? -1.0 : 1.0) * getShooterPivotEncoderPosition() * 360;
    return angle;
  }

  public void driveShooterPivot(double speed) {
    shooterPivotMotors.setAll(speed);
  }

  public void followTarget() {

    vision vision = Robot.getRobotContainer().getVision();
    double pitch = vision.getPitch();
    double target = pitch * HighAltitudeConstants.SHOOTER_PIVOT_PITCH_TO_TARGET_MULTIPLIER
        + HighAltitudeConstants.SHOOTER_PIVOT_PITCH_TO_TARGET_OFFSET;

    SmartDashboard.putNumber("PivotTarget", target);
    setEncoderTarget(target);
    maintainTarget(0.5);

  }

  public void setEncoderTarget(double target) {

    this.encoderTarget = target;

  }

  public double getEncoderTarget() {
    return encoderTarget;
  }

  public void setAngleTarget(double targetDegrees) {

    this.encoderTarget = angleToEncoder(targetDegrees);

  }

  public double angleToEncoder(double angleDegrees) {
    return (RobotMap.SHOOTER_PIVOT_ENCODED_TALON_INVERTED ? -1.0
        : 1.0) * (angleDegrees - HighAltitudeConstants.SHOOTER_PIVOT_ZERO_ANGLE) / 360;
  }

  public boolean maintainTarget(double maxPower) {

    double delta = getEncoderTarget() - getShooterPivotEncoderPosition();
    double power = delta * HighAltitudeConstants.SHOOTER_PIVOT_ANGLE_CORRECTION_CONSTANT;
    power = Math.clamp(power * maxPower, -maxPower, maxPower);

    driveShooterPivot(power);
    return Math.abs(delta) < HighAltitudeConstants.SHOOTER_PIVOT_ARRIVE_OFFSET;
  }

  public void pointToSpeaker(double maxPower) {
    pointToSpeaker(Robot.getRobotContainer().getSwerveDriveTrain().distanceToSpeaker(), maxPower);
  }

  public void pointToSpeaker(double distance, double maxPower) {
    setAngleTarget(distanceToAngle(distance));
    maintainTarget(maxPower);
  }

  public boolean getShooterPivotTopLimitSwitch() {
    if (RobotMap.SHOOTER_PIVOT_TOP_LIMIT_SWITCH_IS_AVAILABLE) {
      return topLimitSwitch.get();
    } else {
      return false;
    }
  }

  public boolean getShooterPivotBottomLimitSwitch() {
    if (RobotMap.SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE) {
      return bottomLimitSwitch.get();
    } else {
      return false;
    }
  }

  public void resetMotorEncoders() {
    shooterPivotMotors.resetEncoder();
  }

  public double getShooterPivotPositionInDegrees() {
    return shooterPivotPositionDegrees;
  }

  public double getCurrentTarget() {
    return currentTarget;
  }

  public void setCurrentTarget(double Target) {
    currentTarget = Target;
  }

  public boolean getOverride() {
    return Override;
  }

  public void toggleOverride() {
    Override = !Override;
  }

  public void resetCanCoder() {
    absoluteEncoderController.setPosition(0);
    // zeroValue = absoluteEncoderController.getPosition().getValueAsDouble();
  }

  public void resetZeroValue() {
    zeroValue = 0;
  }

  public double getShooterPivotRawEncoder() {
    return shooterPivotRawEncoder;
  }

  public double getShooterPivotEncoderPosition() {
    return ShooterPivotEncoderPosition;
  }

  /**
   * Converts the distance (in meters) from the center of the robot to the speaker
   * to an ideal shooter pivot position. This should be mapped at each event with
   * the actual field.
   * 
   * @param distance
   * @return
   */
  public static double distanceToAngle(double distance) {
    if (distance == 0)
      return 0;
    return Math.toDegrees(Math.atan(116 / distance));
  }

  @Override
  public void periodic() {
    ShooterPivotEncoderPosition = absoluteEncoderController.getAbsolutePosition().getValueAsDouble() -
        zeroValue;

    shooterPivotPositionDegrees = getAbsoluteEncoderDeg();
    shooterPivotRawEncoder = absoluteEncoderController.getPosition().getValueAsDouble();

    if (getShooterPivotBottomLimitSwitch())
      resetCanCoder();

    SmartDashboard.putNumber("Shooter Pivot Raw Abs Encoder",
        getShooterPivotRawEncoder());
    SmartDashboard.putNumber("ShooterPivotEncoderTarget", encoderTarget);
    SmartDashboard.putNumber("Shooter Pivot Encoder Position",
        getShooterPivotEncoderPosition());

    SmartDashboard.putBoolean("Shooter_Override", Override);
    SmartDashboard.putNumber("Zero Value shooter pivot", zeroValue);
    SmartDashboard.putBoolean("Shooter_Pivot_Top_Limit_Switch",
        getShooterPivotTopLimitSwitch());
    SmartDashboard.putBoolean("Shooter_Pivot_Bottom_Limit_Switch",
        getShooterPivotBottomLimitSwitch());
  }

}
