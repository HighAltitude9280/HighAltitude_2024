// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;
import frc.robot.subsystems.vision.vision;

public class ShooterPivot extends SubsystemBase {
  HighAltitudeMotorGroup shooterPivotMotors;
  double shooterPivotEncoderPosition, shooterPivotPositionDegrees, shooterPivotRawEncoder;

  DigitalInput topLimitSwitch;
  DigitalInput bottomLimitSwitch;

  CANcoder absoluteEncoderController;

  private double encoderTarget = 0.0;

  private PIDController pidController;

  public ShooterPivot() 
  {

    shooterPivotMotors = new HighAltitudeMotorGroup(RobotMap.SHOOTER_PIVOT_MOTOR_PORTS,
        RobotMap.SHOOTER_PIVOT_INVERTED_MOTORS_PORTS,
        RobotMap.SHOOTER_PIVOT_MOTOR_TYPES);

    shooterPivotMotors.setBrakeMode(HighAltitudeConstants.SHOOTER_PIVOT_MOTOR_BRAKING_MODE);
    shooterPivotMotors.setEncoderInverted(RobotMap.SHOOTER_PIVOT_ENCODER_IS_INVERTED);

    if(HighAltitudeConstants.SHOOTER_PIVOT_CANCODER_AVAILABLE)
    {
      absoluteEncoderController = new CANcoder(RobotMap.SHOOTER_PIVOT_ENCODED_TALON_PORT);
      resetCanCoder();
    }

    if (RobotMap.SHOOTER_PIVOT_TOP_LIMIT_SWITCH_IS_AVAILABLE) {
      topLimitSwitch = new DigitalInput(RobotMap.SHOOTER_PIVOT_TOP_LIMIT_SWITCH_PORT);
    }

    if (RobotMap.SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE) {
      bottomLimitSwitch = new DigitalInput(RobotMap.SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_PORT);
    }

    pidController = new PIDController(HighAltitudeConstants.SHOOTER_PIVOT_KP,
                                      HighAltitudeConstants.SHOOTER_PIVOT_KI,
                                      HighAltitudeConstants.SHOOTER_PIVOT_KD);
}                                   

  public double getShooterPivotPosDegrees() 
  {
     return HighAltitudeConstants.SHOOTER_PIVOT_ZERO_ANGLE +
        (RobotMap.SHOOTER_PIVOT_ENCODED_TALON_INVERTED ? -1.0 : 1.0) 
        * getShooterPivotEncoderPosition() 
        * HighAltitudeConstants.SHOOTER_PIVOT_DEGREES_PER_PULSE;
  }

  public void driveShooterPivot(double speed) 
  {
    shooterPivotMotors.setAll(speed);
  }

  public void followTarget() 
  {

    vision vision = Robot.getRobotContainer().getVision();
    double pitch = vision.getPitch();
    double target = pitch * HighAltitudeConstants.SHOOTER_PIVOT_PITCH_TO_TARGET_MULTIPLIER
        + HighAltitudeConstants.SHOOTER_PIVOT_PITCH_TO_TARGET_OFFSET;

    SmartDashboard.putNumber("PivotTarget", target);
    setEncoderTarget(target);
    maintainTarget(0.5);

  }

  public void setEncoderTarget(double target) 
  {
    this.encoderTarget = target;
  }

  public double getEncoderTarget() 
  {
    return encoderTarget;
  }

  public void setAngleTarget(double targetDegrees) 
  {
    this.encoderTarget = angleToEncoder(targetDegrees);
  }

  public double angleToEncoder(double angleDegrees) 
  {
    return (RobotMap.SHOOTER_PIVOT_ENCODED_TALON_INVERTED ? -1.0: 1.0) 
      * (angleDegrees - HighAltitudeConstants.SHOOTER_PIVOT_ZERO_ANGLE) 
      / HighAltitudeConstants.SHOOTER_PIVOT_DEGREES_PER_PULSE;
  }

  public boolean maintainTarget(double maxPower) 
  {
    double power = pidController.calculate(getShooterPivotEncoderPosition(), getEncoderTarget());
    power = Math.clamp(power * maxPower, -maxPower, maxPower);
    driveShooterPivot(power);

    double delta = getEncoderTarget() - getShooterPivotEncoderPosition();
    return Math.abs(delta) < HighAltitudeConstants.SHOOTER_PIVOT_ARRIVE_OFFSET;
  }

  public void pointToSpeaker(double maxPower) 
  {
    
    pointToSpeaker(Robot.getRobotContainer().getSwerveDriveTrain().distanceToSpeaker(), maxPower);
  }

  public void pointToSpeaker(double distance, double maxPower) 
  {
    setAngleTarget(distanceToAngle(distance));
    maintainTarget(maxPower);
  }

  public void resetMotorEncoders() {
    shooterPivotMotors.resetEncoder();
  }

  public void resetCanCoder() 
  {
    if(HighAltitudeConstants.SHOOTER_PIVOT_CANCODER_AVAILABLE)
      absoluteEncoderController.setPosition(0);
  }

  public double getShooterPivotEncoderPosition() 
  {
    if(HighAltitudeConstants.SHOOTER_PIVOT_CANCODER_AVAILABLE)
      return absoluteEncoderController.getAbsolutePosition().getValueAsDouble();
    else
      return shooterPivotMotors.getEncoderPosition();
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
  public void periodic() 
  {
    SmartDashboard.putNumber("ShooterPivotEncoderTarget", getEncoderTarget());
    SmartDashboard.putNumber("ShooterPivotEncoderPosition",
        getShooterPivotEncoderPosition());

    SmartDashboard.putNumber("ShooterPivotAngle", getShooterPivotPosDegrees());
    SmartDashboard.putNumber("Distance to speaker", 
      Robot.getRobotContainer().getSwerveDriveTrain().distanceToSpeaker());
  }

}
