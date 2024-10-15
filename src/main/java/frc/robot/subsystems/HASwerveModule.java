// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.resources.components.speedController.HighAltitudeMotorController;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;
import frc.robot.resources.math.Math;

/** Add your docs here. */
public class HASwerveModule {
    // another High Altitude Swerve Module Class, this time made by Gomez & Joaquín

    private HighAltitudeMotorController driveMotor;
    private HighAltitudeMotorController directionMotor;

    private boolean isDriveEncoderReversed;
    private boolean isDirectionEncoderReversed;

    private PIDController directionPIDController;
    private SimpleMotorFeedforward directionFeedforward;
    private double currentAngleDirectionPower;

    private PIDController drivePIDController;
    private SimpleMotorFeedforward driveFeedforward;
    private double currentMPSDrivePower;
    private boolean mpsOnTarget = false;

    private double encoderOffSetPulses;

    private boolean isTalonEncoderReversed;
    private CANcoder absoluteEncoderController;

    public HASwerveModule(int driveMotorPort, TypeOfMotor driveTypeOfMotor,
            boolean isDriveMotorReversed, boolean isDriveEncoderReversed,
            int directionMotorPort, TypeOfMotor directionTypeOfMotor,
            boolean isDirectionMotorReversed, boolean isDirectionEncoderReversed,
            int encodedTalonPort, double encoderOffsetPulses, boolean isTalonEncoderReversed) {

        driveMotor = new HighAltitudeMotorController(driveMotorPort, driveTypeOfMotor);
        driveMotor.setInverted(isDriveMotorReversed);
        driveMotor.setBrakeMode(true);
        this.isDriveEncoderReversed = isDriveEncoderReversed;

        directionMotor = new HighAltitudeMotorController(directionMotorPort, directionTypeOfMotor);
        directionMotor.setInverted(isDirectionMotorReversed);
        directionMotor.setBrakeMode(true);
        this.isDirectionEncoderReversed = isDirectionEncoderReversed;

        // DIRECTION CONTROL //
        directionPIDController = new PIDController(0, 0, 0); // 0.128, 0.01, 0.0128
        directionPIDController.enableContinuousInput(-Math.PI, Math.PI);

        directionFeedforward = new SimpleMotorFeedforward(0, 0, 0);

        // DRIVE CONTROL //
        driveFeedforward = new SimpleMotorFeedforward(0, 0);

        drivePIDController = new PIDController(0, 0, 0);
        drivePIDController.enableContinuousInput(-Math.PI, -Math.PI);

        absoluteEncoderController = new CANcoder(encodedTalonPort);
        this.isTalonEncoderReversed = isTalonEncoderReversed;
        this.encoderOffSetPulses = encoderOffsetPulses;

    }

    ///// ENCODERS /////
    public double getAbsoluteEncoderRAD() {
        return (getAbsoluteEncoderRaw() - encoderOffSetPulses) *
                HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE
                * (isTalonEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderRaw() {
        return absoluteEncoderController.getPosition().getValueAsDouble();
    }

    ///// MOTOR ENCODERS /////
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        directionMotor.setEncoderPosition(0);
    }

    public double getDriveEncoder() {
        return driveMotor.getEncPosition() * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveDistance() {
        return driveMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_PULSE
                * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveVelocity() {
        return driveMotor.getEncVelocity() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS
                * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirectionEncoder() {
        return directionMotor.getEncPosition() * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirection() {
        return directionMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_PULSE
                * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    // Getters for the motor objects themselves

    public HighAltitudeMotorController getDriveMotor() {
        return driveMotor;
    }

    public HighAltitudeMotorController getDirectionMotor() {
        return directionMotor;
    }

    // Getters for the position and state of the module

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getAbsoluteEncoderRAD()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRAD()));
    }

    // STATE SETTER

    /*
     * public void setState(SwerveModuleState state) {
     * if (Math.abs(state.speedMetersPerSecond) < 0.01) {
     * stop();
     * return;
     * }
     * 
     * state = SwerveModuleState.optimize(state, getState().angle);
     * 
     * double targetSpeed = state.speedMetersPerSecond;
     * double feedforward = targetSpeed *
     * HighAltitudeConstants.SWERVE_DRIVE_FEEDFORWARD_GAIN;
     * double pidOutput = drivePIDController.calculate(getDriveEncoder(),
     * targetSpeed);
     * //no sé si poner este o getDriveVelocity
     * driveMotor.set(feedforward + pidOutput);
     * 
     * directionMotor.set(directionPIDController.calculate(getAbsoluteEncoderRAD(),
     * state.angle.getRadians()));
     * }
     */

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        directionMotor.set(directionPIDController.calculate(getAbsoluteEncoderRAD(), state.angle.getRadians()));
    }

    public boolean onMPSTarget() {
        return mpsOnTarget;
    }

    public void setMPSPower(double power) {
        currentMPSDrivePower = power;
    }

    public void driveMotor(double speed) {
        driveMotor.set(speed);
    }

    public boolean swerveDriveMPS(int mps) {
        double delta = mps - getDriveVelocity();
        this.currentMPSDrivePower += delta * HighAltitudeConstants.SWERVE_MPS_STEP;
        driveMotor(currentMPSDrivePower);

        SmartDashboard.putNumber(" Swerve Drive MPS ", currentMPSDrivePower);

        mpsOnTarget = (Math.abs(delta) <= HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
        return mpsOnTarget;
    }

    public boolean controlSwerveDrive(int mps) {
        double driveOutput = driveFeedforward.calculate(mps);
        driveOutput += drivePIDController.calculate(getDriveVelocity(), mps);
        driveMotor(driveOutput);

        double delta = mps - getDriveVelocity();

        mpsOnTarget = (Math.abs(delta) <= HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
        SmartDashboard.putBoolean("Swerve onTarget", Math.abs(delta) <= HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
        return mpsOnTarget;
    }

    public PIDController getDirectionPIDController() {
        return directionPIDController;
    }

    public PIDController getDrivePIDController() {
        return drivePIDController;
    }

    public void stop() {
        driveMotor.set(0);
        directionMotor.set(0);
    }

}
