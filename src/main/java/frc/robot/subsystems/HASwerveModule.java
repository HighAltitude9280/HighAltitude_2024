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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.resources.components.speedController.HighAltitudeMotorController;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;
import frc.robot.resources.math.Math;
import edu.wpi.first.math.controller.ProfiledPIDController;

/** Add your docs here. */
public class HASwerveModule {
    // another High Altitude Swerve Module Class, this time made by Gomez & Joaquín
    /// DIRECTION ///
    private HighAltitudeMotorController directionMotor;
    private boolean isDirectionEncoderReversed;

    private final ProfiledPIDController directionProfiledPIDController;
    private TrapezoidProfile.Constraints swConstraints;

    private double directionLastSpeed = 0;
    private double lastTime = Timer.getFPGATimestamp();

    private SimpleMotorFeedforward directionFeedforward;
    private double currentAngleDirectionPower;

    /// DRIVE ///
    private HighAltitudeMotorController driveMotor;
    private boolean isDriveEncoderReversed;

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
        // 0.128, 0.01, 0.0128
        directionProfiledPIDController = new ProfiledPIDController(HighAltitudeConstants.SWERVE_DIRECTION_kP,
                HighAltitudeConstants.SWERVE_DIRECTION_kI, HighAltitudeConstants.SWERVE_DIRECTION_kD,
                new TrapezoidProfile.Constraints(HighAltitudeConstants.SWERVE_DIRECTION_MAX_VELOCITY,
                        HighAltitudeConstants.SWERVE_DIRECTION_MAX_ACCELERATION));

        directionProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        directionFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SWERVE_DIRECTION_kS,
                HighAltitudeConstants.SWERVE_DIRECTION_kV,
                HighAltitudeConstants.SWERVE_DIRECTION_kA);

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

    public double getEncoderVelocity() {
        return absoluteEncoderController.getVelocity().getValueAsDouble() * 2 * Math.PI
                * (isTalonEncoderReversed ? -1.0 : 1.0);

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
     * directionMotor.set(directionProfiledPIDController.calculate(
     * getAbsoluteEncoderRAD(),
     * state.angle.getRadians()));
     * }
     */

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        controlSwerveDirection(state.angle.getRadians());
        controlSwerveSpeed(state.speedMetersPerSecond);
    }

    public boolean onMPSTarget() {
        return mpsOnTarget;
    }

    public void setMPSPower(double power) {
        currentMPSDrivePower = power;
    }

    public void controlSwerveSpeed(double mps) {
        double driveOutput = driveFeedforward.calculate(mps);
        driveOutput += drivePIDController.calculate(getDriveVelocity(), mps);
        driveMotor.setVoltage(driveOutput);

        double delta = mps - getDriveVelocity();

        mpsOnTarget = (Math.abs(delta) <= HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
        // SmartDashboard.putBoolean("Drive Swerve onTarget", Math.abs(delta) <=
        // HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
        return;
    }

    public void controlSwerveDirection(double angleTarget) {
        double pidVal = directionProfiledPIDController.calculate(getDirectionEncoder(),
                angleTarget);
        double targetSpeed = directionProfiledPIDController.getSetpoint().velocity;

        double directionAcceleration = (targetSpeed - directionLastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        double feedforwardVal = directionFeedforward.calculate(targetSpeed, directionAcceleration);

        double directionOutput = pidVal + feedforwardVal;

        directionMotor.setVoltage(directionOutput);
        directionLastSpeed = targetSpeed;
        lastTime = Timer.getFPGATimestamp();

        double delta = currentAngleDirectionPower - getAbsoluteEncoderRAD();

        // SmartDashboard.putBoolean("Direction Swerve onTarget", Math.abs(delta) <=
        // HighAltitudeConstants.SWERVE_DIRECTION_ON_TARGET);
    }

    public ProfiledPIDController getDirectionPIDController() {
        return directionProfiledPIDController;
    }

    public PIDController getDrivePIDController() {
        return drivePIDController;
    }

    public void stop() {
        driveMotor.set(0);
        directionMotor.set(0);
    }

    // Smartdashboard prints for debugging.

    public void putRawEncoderValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveEncPos", driveMotor.getEncPosition());
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
        SmartDashboard.putNumber(identifier + "AbsPos", getAbsoluteEncoderRAD());
        SmartDashboard.putNumber(identifier + "AbsRawPos", absoluteEncoderController.getPosition().getValueAsDouble());
    }

    public void putMotorOutputs(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveOut", driveMotor.getOutput());
        SmartDashboard.putNumber(identifier + "DirOut", directionMotor.getOutput());
    }

    public void putTestPID(String identifier, SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        SmartDashboard.putNumber(identifier + "PID", state.angle.getRadians());
    }
}