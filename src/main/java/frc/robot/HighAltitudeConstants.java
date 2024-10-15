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

/** Add your docs here. */
public class HighAltitudeConstants {

        ////////////////////////// SWERVE //////////////////////////

        public static final double MAX_VOLTAGE = 12;

        ///   CONSTANTS FOR MK4i L4 Config DRIVE MOTOR  ///
        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4 * 0.0254;
        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver)                                               //pinion
        public static final double SWERVE_DRIVE_GEAR_RATIO = (50.0 * 16.0 * 45.0) / ( 16.0  * 28.0 * 15.0);
                                                                             //ft/s   //ft -> in  //im -> m
        public static final double SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND = 20.4  *     12   *   0.0254;

        /////////// KINEMATICS
        // Distance left - right (meters)
        public static final double SWERVE_TRACK_WIDTH = 26 * 0.0254;
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
        public static final double SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND = 15.0;
        public static final double SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 15.0;

        // Other

        public static final double SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION = 1f;
        // encoder * this value = radians
        public static final double SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE = (2.0 * Math.PI)
                        / SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION;

        ///////////   DRIVING MOTOR   ///////////

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DRIVE_PULSES_PER_REVOLUTION = 1.0;
        public static final double SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_METERS_PER_PULSE = (Math.PI * SWERVE_WHEEL_DIAMETER)
                        / (SWERVE_DRIVE_PULSES_PER_REVOLUTION * SWERVE_DRIVE_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = meters/second
        public static final double SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS = (SWERVE_DRIVE_METERS_PER_PULSE)
                        / 60;

         public static final double SWERVE_DRIVE_FEEDFORWARD_GAIN = 0;

        

        // Arbitrary to make controlling the swerve easier in teleop
        /*public static final double SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND = SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
                        * 0.8;*/

        /////////// DIRECTION MOTOR

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

        /*public static final double SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI * 0.75;*/

        //// DIRECTION PID
        public static final double SWERVE_DIRECTION_BRAKING_RADIANS = (Math.PI * 2) / 4; // 2pi/3
        public static final double SWERVE_DIRECTION_KP = 0.0;
        public static final double SWERVE_DIRECTION_KD = 0.0;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                        new PIDConstants(0.9, 0, 0.000025), // Translation constants
                        new PIDConstants(2.0, 0, 0), // Rotation constants
                        SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND,
                        Math.hypot(SWERVE_TRACK_WIDTH / 2, SWERVE_WHEEL_BASE / 2), // Drive base radius (distance from
                                                                                   // center to
                                                                                   // furthest module)
                        new ReplanningConfig());

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 0; //32.0;
        public static final double SWERVE_TURN_ARRIVE_OFFSET = 0; //3.0;

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

        // Order of tuning: feed forward, then PID

        public static final double SHOOTER_TOP_kP = 0.00005;
        public static final double SHOOTER_TOP_kD = 0.000011;

        public static final double SHOOTER__TOP_kS = 0.0;
        public static final double SHOOTER__TOP_kV = 0.000177;

        public static final double SHOOTER_BOTTOM_kP = 0.000027;
        public static final double SHOOTER_BOTTOM_kD = 0.000011;

        public static final double SHOOTER_BOTTOM_kS = 0.0;
        public static final double SHOOTER_BOTTOM_kV = 0.0001835;

        ////////////////////////// INTAKE //////////////////////////

        public static final boolean INTAKE_MOTOR_BRAKING_MODE = false;

        ////////////////////////// SHOOTER PIVOT //////////////////////////

        public static final boolean SHOOTER_PIVOT_MOTOR_BRAKING_MODE = true;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. SHOOTER PIVOT REVS / MOTOR REVS
        public static final double SHOOTER_PIVOT_RATIO = ((72.0 * 80.0 * 26.0) / (12.0 * 16.0 * 12.0));
        // 2592 //124416
        public static final double SHOOTER_PIVOT_DEGREES_PER_REVOLUTION = 360
                        / SHOOTER_PIVOT_RATIO;

        public static final double SHOOTER_PIVOT_BRAKING_DEGREES = 0; //50;

        public static final double SHOOTER_PIVOT_ARRIVE_OFFSET = 0; //0.05;

        public static final double SHOOTER_PIVOT_ABSOLUTE_ENCODER_DEGREES_PER_PULSE = 360
                        / SHOOTER_PIVOT_RATIO;

        public static final double SHOOTER_PIVOT_AUTO_MAX_POWER = 0.5;

        public static final double SHOOTER_PIVOT_UPPER_LIMIT = 0; //107.7;
        // 107.666015625 Shuffle Report

        public static final double SHOOTER_PIVOT_LOWER_LIMIT = 0.0;
        // 0.17578125 Shuffle Report

        public static final double SHOOTER_PIVOT_ANGLE_CORRECTION_CONSTANT = 0; //20;

        public static final double SHOOTER_PIVOT_PITCH_TO_TARGET_MULTIPLIER =0; //-0.003;
        public static final double SHOOTER_PIVOT_PITCH_TO_TARGET_OFFSET = 0; //.18;

        public static final double SHOOTER_PIVOT_ZERO_ANGLE = 0; //65.0;


        //////////////////////// DRIVERS ////////////////////////

        ////////////////////////////// Pathfinding ////////////////////////}

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 1;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 1.5;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI / 2;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = Math.PI;

        public static final Pose3d SPEAKER = new Pose3d(0f, 1.6, 1.64, new Rotation3d());
        public static final Pose2d AMP_POS = new Pose2d(new Translation2d(3., 4.31), new Rotation2d(-Math.PI / 2));

        public static final double SWERVE_DRIVE_ON_TARGET = 0;

        public static final double SWERVE_MPS_STEP = 0;

        public static final double SWERVE_DIRECTION_ON_TARGET = 0;


        ///     PROFILED PID CONTROLLER         ///

        //      CONSTRAINTS     //
        public static final double SWERVE_DIRECTION_MAX_VELOCITY = 0;
        public static final double SWERVE_DIRECTION_MAX_ACCELERATION = 0;

        //      FEEDBACK     //
        public static final double SWERVE_DIRECTION_kP = 0;
        public static final double SWERVE_DIRECTION_kI = 0;
        public static final double SWERVE_DIRECTION_kD = 0;

        //      FEEDFORWARD     //
        public static final double SWERVE_DIRECTION_kS = 0;
        public static final double SWERVE_DIRECTION_kV = 0;
        public static final double SWERVE_DIRECTION_kA = 0;


        

}
