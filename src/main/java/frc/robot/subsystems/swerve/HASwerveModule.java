// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import javax.swing.LayoutStyle;

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

    private double directionPIDAngleSetPoint = 0;
    private double directionPIDVelocitySetPoint = 0;
    private double directionPIDAccelerationSetPoint = 0;

    private double lastEncoderVelocityEnc = 0;
    private double lastTimeEnc = Timer.getFPGATimestamp();

    private double directionLastSpeedAcc = 0;
    private double lastTimeAcc = Timer.getFPGATimestamp();

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

        // enableContinousInput() calculates the route with less error
        directionProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        directionFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SWERVE_DIRECTION_kS,
                HighAltitudeConstants.SWERVE_DIRECTION_kV,
                HighAltitudeConstants.SWERVE_DIRECTION_kA);

        // DRIVE CONTROL //
        driveFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SWERVE_DRIVE_kS,
                HighAltitudeConstants.SWERVE_DRIVE_kV);

        drivePIDController = new PIDController(HighAltitudeConstants.SWERVE_DRIVE_kP,
                HighAltitudeConstants.SWERVE_DRIVE_kI, HighAltitudeConstants.SWERVE_DRIVE_kD);

        absoluteEncoderController = new CANcoder(encodedTalonPort);
        this.isTalonEncoderReversed = isTalonEncoderReversed;
        this.encoderOffSetPulses = encoderOffsetPulses;

    }

    ///// CANCODER /////

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

    public double getEncoderAcceleration() {
        return (lastEncoderVelocityEnc - getEncoderVelocity()) / lastTimeEnc - Timer.getFPGATimestamp();
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
        Math.clamp(driveOutput, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);
        driveMotor.setVoltage(driveOutput);

        double delta = mps - getDriveVelocity();

        mpsOnTarget = (Math.abs(delta) <= HighAltitudeConstants.SWERVE_DRIVE_ON_TARGET);
    }

    public void controlSwerveDirection(double angleTarget) {
        double pidVal = directionProfiledPIDController.calculate(getDirectionEncoder(),
                angleTarget);
        double targetSpeed = directionProfiledPIDController.getSetpoint().velocity;

        double directionAcceleration = (targetSpeed - directionLastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        double feedforwardVal = directionFeedforward.calculate(targetSpeed, directionAcceleration);

        double directionOutput = pidVal + feedforwardVal;
        Math.clamp(directionOutput, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);

        directionMotor.setVoltage(directionOutput);
        directionLastSpeed = targetSpeed;
        lastTime = Timer.getFPGATimestamp();

        directionPIDAccelerationSetPoint = (targetSpeed - directionLastSpeedAcc)
                / (Timer.getFPGATimestamp() - lastTimeAcc);
        directionPIDAngleSetPoint = getDirectionPIDController().getSetpoint().position;
        directionPIDVelocitySetPoint = getDirectionPIDController().getSetpoint().velocity;

        getEncoderAcceleration();
        double delta = currentAngleDirectionPower - getAbsoluteEncoderRAD();
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

    public void putProcessedValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DrivePos", getDriveDistance());
        SmartDashboard.putNumber(identifier + "DirPos", getDirection());
        SmartDashboard.putNumber(identifier + "AbsPos", getAbsoluteEncoderRAD());
        SmartDashboard.putNumber(identifier + "AbsRawPos", absoluteEncoderController.getPosition().getValueAsDouble());
    }

    public void controlTunning(String identifier) {
        // This is what you should print:
        // 1. Velocity of the DriveMotorEnc
        SmartDashboard.putNumber(identifier, driveMotor.getEncVelocity());

        // 2. Graphic of the CANCoder Angle
        SmartDashboard.putNumber(identifier, getAbsoluteEncoderRAD());

        // 3. Graphic of the CANCoder Velocity
        SmartDashboard.putNumber(identifier, getEncoderVelocity());

        // 4. Graphic of the CANCoder Acceleration
        SmartDashboard.putNumber(identifier, getEncoderAcceleration());

        // 5. Setpoint of the ProfiledPIDController Angle
        SmartDashboard.putNumber(identifier, directionPIDAngleSetPoint);

        // 6. Setpoint of the ProfiledPIDController Velocity
        SmartDashboard.putNumber(identifier, directionPIDVelocitySetPoint);

        // 7. Setpoint of the ProfiledPIDController Acceleration
        SmartDashboard.putNumber(identifier, directionPIDAccelerationSetPoint);

    }
}