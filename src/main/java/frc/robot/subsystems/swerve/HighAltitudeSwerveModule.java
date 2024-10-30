// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.resources.math.Math;
import frc.robot.resources.components.speedController.HighAltitudeMotorController;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;
import frc.robot.resources.components.speedController.TalonFX.SuperTalonFX;

/** Add your docs here. */
public class HighAltitudeSwerveModule {
    private HighAltitudeMotorController directionMotor;

    private boolean isDirectionEncoderReversed;
    private boolean isTalonEncoderReversed;

    private PIDController directionPIDController;

    /// DRIVE ///
    private SuperTalonFX driveMotor;
    private boolean isDriveEncoderReversed;

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveAppliedVolts;
    private StatusSignal<Double> driveAcceleration;
    private StatusSignal<Double> driveCurrent;

    TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();

    private CANcoder absoluteEncoderController;
    private double encoderOffsetPulses;

    public HighAltitudeSwerveModule(int driveMotorPort, TypeOfMotor driveTypeOfMotor, boolean isDriveMotorReversed,
            boolean isDriveEncoderReversed,
            int directionMotorPort, TypeOfMotor directionTypeOfMotor,
            boolean isDirectionMotorReversed, boolean isDirectionEncoderReversed, int encodedTalonPort,
            double encoderOffsetPulses, boolean isTalonEncoderReversed) {

        driveMotor = new SuperTalonFX(driveMotorPort, isDriveMotorReversed, driveMotorConfiguration);
        driveMotor.setInverted(isDriveMotorReversed);
        this.isDriveEncoderReversed = isDriveEncoderReversed;

        directionMotor = new HighAltitudeMotorController(directionMotorPort, directionTypeOfMotor);
        directionMotor.setInverted(isDirectionMotorReversed);
        directionMotor.setBrakeMode(true);
        this.isDirectionEncoderReversed = isDirectionEncoderReversed;

        directionPIDController = new PIDController(0.128, 0.01, 0.0128); // 125, 01, 0125
        directionPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoderController = new CANcoder(encodedTalonPort);
        this.isTalonEncoderReversed = isTalonEncoderReversed;
        this.encoderOffsetPulses = encoderOffsetPulses;

        // DRIVE MOTOR //

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveAcceleration = driveMotor.getAcceleration();
        driveCurrent = driveMotor.getSupplyCurrent();

    }

    // este es el bueno
    public double getAbsoluteEncoderRad() {
        /*
         * double angleRadians =
         * absoluteEncoderController.getPosition().getValueAsDouble()
         * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE;
         * angleRadians -= encoderOffsetPulses *
         * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE;
         * return angleRadians * (isTalonEncoderReversed ? -1.0 : 1.0);
         */
        double angle = absoluteEncoderController.getPosition().getValueAsDouble() - encoderOffsetPulses;
        return angle * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE
                * (isTalonEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
    }

    // Getters for encoder values and velocities

    public double getDriveEncoder() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveDistance() {
        return driveMotor.getPosition().getValueAsDouble() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_REVOLUTION;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble()
                * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS;
    }

    public double getDirectionEncoder() {
        return directionMotor.getEncPosition() * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirection() {
        return directionMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_PULSE
                * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirectionVelocity() {
        return directionMotor.getEncVelocity()
                * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS
                * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    // Getters for the motor objects themselves

    public SuperTalonFX getDriveMotor() {
        return driveMotor;
    }

    public HighAltitudeMotorController getDirectionMotor() {
        return directionMotor;
    }

    // Getters for the position and state of the module

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    // STATE SETTER

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
        directionMotor.set(directionPIDController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public PIDController getPIDController() {
        return directionPIDController;
    }

    public void setStateRegardlessOfSpeed(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
        directionMotor.set(directionPIDController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        directionMotor.set(0);
    }

    // Smartdashboard prints for debugging.

    public void putRawEncoderValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveEncPos", driveMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(identifier + "DirEncPos", directionMotor.getEncPosition());
        SmartDashboard.putNumber(identifier + "AbsEncPos", absoluteEncoderController.getPosition().getValueAsDouble());
    }

    public void putEncoderValuesInvertedApplied(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveEncPos", getDriveEncoder());
        SmartDashboard.putNumber(identifier + "DirEncPos", getDirectionEncoder());
        SmartDashboard.putNumber(identifier + "AbsEncPos",
                absoluteEncoderController.getPosition().getValueAsDouble() * (isTalonEncoderReversed ? -1.0 : 1.0));
    }

    public void putProcessedValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DrivePos", getDriveDistance());
        SmartDashboard.putNumber(identifier + "DirPos", getDirection());
        SmartDashboard.putNumber(identifier + "AbsPos", getAbsoluteEncoderRad());
        SmartDashboard.putNumber(identifier + "AbsRawPos", absoluteEncoderController.getPosition().getValueAsDouble());
    }

    public void putMotorOutputs(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveOut", driveMotor.getBridgeOutput().getValueAsDouble());
        SmartDashboard.putNumber(identifier + "DirOut", directionMotor.getOutput());
    }

    public void putTestPID(String identifier, SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        SmartDashboard.putNumber(identifier + "PID", state.angle.getDegrees());
    }
}
