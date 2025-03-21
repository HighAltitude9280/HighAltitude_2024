// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Human_Drivers.HumanDrivers;

/** Add your docs here. */
public class HighAltitudeConstants {

        ////////////////////////// SWERVE //////////////////////////

        public static final double MAX_VOLTAGE = 12.5;
        // ponlo a 3 para pruebas

        /// CONSTANTS FOR MK4i L4 Config DRIVE MOTOR ///
        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4 * 0.0254;
        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver) //pinion
        public static final double SWERVE_DRIVE_GEAR_RATIO = (50.0 * 16.0 * 45.0) / (16.0 * 28.0 * 15.0);
        // ft/s //ft -> in //im -> m
        public static final double SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND = 19.5 * 12 * 0.0254;

        /////////// KINEMATICS
        // Distance left - right (meters)
        public static final double SWERVE_TRACK_WIDTH = 26 * 0.0254; // este es de llanta a llanta
        // Distance front - back (meters)
        public static final double SWERVE_WHEEL_BASE = 23 * 0.0254;

        // FL, FR, BL, BR. Remember these cartesian coordinates consider the x axis to
        // be headed where the robot is pointing to. The y-axis direction could be a
        // source of problems...
        // WPILib says "Positive x values represent moving toward the front of the robot
        // whereas positive y values represent moving toward the left of the robot."
        // The example I saw uses the raw yaw reported by the navx and switches the
        // position of the left and right wheels in the kinematics.
        // I will use CCW and the allegedly correct x y coordinates.
        // For some reason, that did not work. The kinematics seem to work correctly
        // when "left" is negative
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2));

        // Arbitrary. Higher numbers will cause the swerve to react more violently to
        // joysitck inputs and may not be ideal. Lower numbers will cause the swerve to
        // have a very slow reaction to joystick inputs, and may not be ideal.
        public static final double SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND = 20.0;
        public static final double SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 20.0;

        // Other

        public static final double SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION = 1f;
        // encoder * this value = radians
        public static final double SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE = (2.0 * Math.PI)
                        / SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION;

        /////////// DRIVING MOTOR ///////////

        // HOW TO GET THE VALUES //
        /*
         * Necesitas la gráfica de velocidad del encoder del driveMotor
         * 
         * PASO 1:
         * 1. PID en 0
         * 2. kS dejarla en 0
         * 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2:
         * 4. Ya no mueves el feedforward
         * 5. Poner la kP lo más grande que pueda sin que se pase del target
         * 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños,
         * que quede smooth
         */

        // FEEDBACK //
        public static final double SWERVE_DRIVE_kP = 1.7375;
        public static final double SWERVE_DRIVE_kI = 0;
        public static final double SWERVE_DRIVE_kD = 0.0;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_METERS_PER_REVOLUTION = (Math.PI * SWERVE_WHEEL_DIAMETER)
                        / (SWERVE_DRIVE_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = meters/second
        public static final double SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS = SWERVE_DRIVE_METERS_PER_REVOLUTION;

        public static final double SWERVE_VELOCITY_IN_METERS_PER_SEC = (10980 / 2048)
                        * (SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS);

        public static final double SWERVE_DRIVE_CLEANUP_MODE_SPEED_METERS_PER_SECOND = SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
                        * 0.8;

        // Arbitrary to make controlling the swerve easier in teleop
        /*
         * public static final double SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND =
         * SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
         * 0.8;
         */

        /////////// DIRECTION MOTOR ///////////

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DIRECTION_PULSES_PER_REVOLUTION = 1.0;
        public static final double SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver)
        public static final double SWERVE_DIRECTION_GEAR_RATIO = 150.0 / 7.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = radians
        public static final double SWERVE_DIRECTION_RADIANS_PER_PULSE = Math.PI * 2
                        / (SWERVE_DIRECTION_PULSES_PER_REVOLUTION * SWERVE_DIRECTION_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = radians/second
        public static final double SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS = (1000
                        * SWERVE_DIRECTION_RADIANS_PER_PULSE)
                        / SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS;

        /*
         * public static final double
         * SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI *
         * 0.75;
         */

        //// DIRECTION PID ////

        /// PROFILED PID CONTROLLER FOR SWERVE DIRECTION ///

        // CONSTRAINTS //
        public static final double SWERVE_DIRECTION_MAX_VELOCITY = 0;
        public static final double SWERVE_DIRECTION_MAX_ACCELERATION = 0;

        // HOW TO GET THE VALUES //
        /*
         * Necesitas las graficas: a) Gráfica del angúlo
         * b) Gráfica de velocidad del encoder
         * c) Gráfica de aceleración
         * d) Setpoint del ángulo
         * e) Setpoint de la velocidad
         * f) Setpoint de aceleración
         * 
         * PASO 1:
         * 1. PID en 0
         * 2. kS dejarla en 0
         * 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2:
         * 4. Ya no mueves el feedforward
         * 5. Poner la kP lo más grande que pueda sin que se pase del target
         * 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños,
         * que quede smooth
         */
        // FEEDBACK //

        public static final double SWERVE_DIRECTION_kP = 0.0; // 0.128
        public static final double SWERVE_DIRECTION_kI = 0; // 0.01
        public static final double SWERVE_DIRECTION_kD = 0; // 0.0128

        // FEEDFORWARD //
        public static final double SWERVE_DIRECTION_kS = 0;
        public static final double SWERVE_DIRECTION_kV = 0.01;
        public static final double SWERVE_DIRECTION_kA = 0; // no sé si dejarlo en 0

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                        new PIDConstants(0.9, 0, 0.000025), // Translation constants
                        new PIDConstants(2.0, 0, 0), // Rotation constants
                        SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND,
                        Math.hypot(SWERVE_TRACK_WIDTH / 2, SWERVE_WHEEL_BASE / 2), // Drive base radius (distance from
                                                                                   // center to
                                                                                   // furthest module)
                        new ReplanningConfig());

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 32; // 32.0;
        public static final double SWERVE_TURN_ARRIVE_OFFSET = 3; // 3.0;

        //// VISION

        public static final double YAW_CORRECTION = 0.15;
        public static final double YAW_OFFSET = 5.72;

        public static final double DISTANCE_CORRECTION = 0.5;

        public static final double NOTE_DETECTION_YAW_OFFSET = -2.0;
        public static final double NOTE_DETECTION_YAW_P = 08.0;

        public static final double NOTE_DETECTION_DRIVE_POWER = 0.3;
        public static final double NOTE_DETECTION_TURN_POWER = 0.8;

        ////////////////////////// SHOOTER //////////////////////////

        public static final boolean SHOOTER_MOTORS_BRAKING_MODE = false;
        public static final double SHOOTER_RPM_TO_POWER = 1f / 5000f;
        public static final double SHOOTER_RPM_STEP = 0.000004;
        public static final double SHOOTER_ON_TARGET = 50;

        // LEFT //

        // Order of tuning: feed forward, then PID
        public static final double SHOOTER_LEFT_kP = 0.00005;
        public static final double SHOOTER_LEFT_kD = 0.000011;

        public static final double SHOOTER_LEFT_kS = 0.0;
        public static final double SHOOTER_LEFT_kV = 0.000177;

        // RIGHT //
        public static final double SHOOTER_RIGHT_kP = 0.000027;
        public static final double SHOOTER_RIGHT_kD = 0.000011;

        public static final double SHOOTER_RIGHT_kS = 0.0;
        public static final double SHOOTER_RIGHT_kV = 0.0001835;

        ////////////////////////// INDEXER //////////////////////////

        public static final double INDEXER_IN_SPEED = 0.7;
        public static final double INDEXER_OUT_SPEED = -0.7;

        ////////////////////////// INTAKE //////////////////////////

        public static final double INTAKE_IN_SPEED = 0.6;
        public static final double INTAKE_OUT_SPEED = -0.6;

        public static final boolean INTAKE_MOTOR_BRAKING_MODE = false;

        ////////////////////////// SHOOTER PIVOT //////////////////////////

        public static final boolean SHOOTER_PIVOT_MOTOR_BRAKING_MODE = true;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. SHOOTER PIVOT REVS / MOTOR REVS
        public static final double SHOOTER_PIVOT_RATIO = ((72.0 * 80.0 * 26.0) / (12.0 * 16.0 * 12.0));
        // 2592 //124416
        public static final double SHOOTER_PIVOT_DEGREES_PER_REVOLUTION = 360
                        / SHOOTER_PIVOT_RATIO;

        public static final double SHOOTER_PIVOT_BRAKING_DEGREES = 0; // 50;

        public static final double SHOOTER_PIVOT_ARRIVE_OFFSET = 0; // 0.05;

        public static final double SHOOTER_PIVOT_ABSOLUTE_ENCODER_DEGREES_PER_PULSE = 360
                        / SHOOTER_PIVOT_RATIO;

        public static final double SHOOTER_PIVOT_AUTO_MAX_POWER = 0.5;

        public static final double SHOOTER_PIVOT_UPPER_LIMIT = 0; // 107.7;
        // 107.666015625 Shuffle Report

        public static final double SHOOTER_PIVOT_LOWER_LIMIT = 0.0;
        // 0.17578125 Shuffle Report

        public static final double SHOOTER_PIVOT_ANGLE_CORRECTION_CONSTANT = 0; // 20;

        public static final double SHOOTER_PIVOT_PITCH_TO_TARGET_MULTIPLIER = 0; // -0.003;
        public static final double SHOOTER_PIVOT_PITCH_TO_TARGET_OFFSET = 0; // .18;

        public static final double SHOOTER_PIVOT_ZERO_ANGLE = 0; // 65.0;

        //////////////////////// DRIVERS ////////////////////////

        public static final HumanDrivers CURRENT_PILOT = HumanDrivers.Joakin;
        public static final HumanDrivers CURRENT_COPILOT = HumanDrivers.Joakin;

        ////////////////////////////// Pathfinding ////////////////////////}

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 1;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 1.5;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI / 2;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = Math.PI;

        public static final Pose3d SPEAKER = new Pose3d(0f, 1.6, 1.64, new Rotation3d());
        public static final Pose2d AMP_POS = new Pose2d(new Translation2d(3., 4.31), new Rotation2d(-Math.PI / 2));

        public static final double SWERVE_DRIVE_ON_TARGET = 0;

        public static final Pose2d CURRENT_ROBOT_POSE = null;

}
