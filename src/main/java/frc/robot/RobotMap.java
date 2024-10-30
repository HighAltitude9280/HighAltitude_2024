// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;

/** Add your docs here. */
public class RobotMap {

    ////////////////////////// SWERVE //////////////////////////

    ///// FRONT LEFT
    // DRIVE
    public static final int SWERVE_FRONT_LEFT_DRIVE_MOTOR_PORT = 16;
    public static final TypeOfMotor SWERVE_FRONT_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean SWERVE_FRONT_LEFT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_FRONT_LEFT_DIRECTION_MOTOR_PORT = 17;
    public static final TypeOfMotor SWERVE_FRONT_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
    public static final boolean SWERVE_FRONT_LEFT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_LEFT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_FRONT_LEFT_ENCODED_TALON_PORT = 39;
    public static final double SWERVE_FRONT_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = 0.051025390625;
    public static final boolean SWERVE_FRONT_LEFT_ENCODED_TALON_INVERTED = false;

    ///// FRONT RIGHT
    // DRIVE
    public static final int SWERVE_FRONT_RIGHT_DRIVE_MOTOR_PORT = 10;
    public static final TypeOfMotor SWERVE_FRONT_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_RIGHT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_PORT = 11;
    public static final TypeOfMotor SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
    public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_FRONT_RIGHT_ENCODED_TALON_PORT = 40;
    public static final double SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = 0.086181640625;
    public static final boolean SWERVE_FRONT_RIGHT_ENCODED_TALON_INVERTED = false;

    ///// BACK LEFT
    // DRIVE
    public static final int SWERVE_BACK_LEFT_DRIVE_MOTOR_PORT = 14;
    public static final TypeOfMotor SWERVE_BACK_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean SWERVE_BACK_LEFT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_BACK_LEFT_DIRECTION_MOTOR_PORT = 15;
    public static final TypeOfMotor SWERVE_BACK_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
    public static final boolean SWERVE_BACK_LEFT_DIRECTION_MOTOR_INVERTED = false;
    public static final boolean SWERVE_BACK_LEFT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_BACK_LEFT_ENCODED_TALON_PORT = 38;
    public static final double SWERVE_BACK_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = 0.2158203125;
    public static final boolean SWERVE_BACK_LEFT_ENCODED_TALON_INVERTED = false;

    ///// BACK RIGHT
    // DRIVE
    public static final int SWERVE_BACK_RIGHT_DRIVE_MOTOR_PORT = 12;
    public static final TypeOfMotor SWERVE_BACK_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean SWERVE_BACK_RIGHT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_BACK_RIGHT_DIRECTION_MOTOR_PORT = 13;
    public static final TypeOfMotor SWERVE_BACK_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
    public static final boolean SWERVE_BACK_RIGHT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_BACK_RIGHT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_BACK_RIGHT_ENCODED_TALON_PORT = 37;
    public static final double SWERVE_BACK_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = -0.116943359375;
    public static final boolean SWERVE_BACK_RIGHT_ENCODED_TALON_INVERTED = false;

    ////////////////////////// SHOOTER //////////////////////////

    public static final int[] SHOOTER_LEFT_MOTOR_PORTS = { 30 };
    public static final int[] SHOOTER_LEFT_INVERTED_MOTORS_PORTS = {};
    public static final boolean SHOOTER_LEFT_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] SHOOTER_LEFT_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

    public static final int[] SHOOTER_RIGHT_MOTOR_PORTS = { 31 };
    public static final int[] SHOOTER_RIGHT_INVERTED_MOTORS_PORTS = {};
    public static final boolean SHOOTER_RIGHT_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] SHOOTER_RIGHT_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

    public static final int[] SHOOTER_INDEXER_MOTOR_PORTS = { 32 };
    public static final int[] SHOOTER_INDEXER_INVERTED_MOTORS_PORTS = {};
    public static final boolean SHOOTER_INDEXER_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] SHOOTER_INDEXER_MOTOR_TYPES = { TypeOfMotor.TALON_SRX };

    public static final double SHOOTER_HEIGHT = 0;
    public static final int SHOOTER_PROXIMITY_SENSOR_PORT = 1;

    ////////////////////////// INTAKE //////////////////////////

    public static final int[] INTAKE_MOTOR_PORTS = { 20 };
    public static final int[] INTAKE_INVERTED_MOTORS_PORTS = { 20 };
    public static final boolean INTAKE_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] INTAKE_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };
    public static final int INTAKE_ANALOG_SENSOR_PORT = 0;

    ////////////////////////// PIVOTS //////////////////////////

    ///// SHOOTER PIVOT
    public static final int[] SHOOTER_PIVOT_MOTOR_PORTS = { 41 };
    public static final int[] SHOOTER_PIVOT_INVERTED_MOTORS_PORTS = {};
    public static final boolean SHOOTER_PIVOT_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] SHOOTER_PIVOT_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

    // CANCODER
    public static final int SHOOTER_PIVOT_ENCODED_TALON_PORT = 45;
    public static final double SHOOTER_PIVOT_ENCODER_OFFSET_PULSES = 0;
    public static final boolean SHOOTER_PIVOT_ENCODED_TALON_INVERTED = false;

    // LIMIT SWITCHES
    public static final boolean SHOOTER_PIVOT_TOP_LIMIT_SWITCH_IS_AVAILABLE = false;
    public static final boolean SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE = false;

    public static final int SHOOTER_PIVOT_TOP_LIMIT_SWITCH_PORT = 9;
    public static final int SHOOTER_PIVOT_BOTTOM_LIMIT_SWITCH_PORT = 8;

    ////////////////////////// CLIMBER //////////////////////////
    public static final int[] CLIMBER_MOTOR_PORTS = { 51, 52 };
    public static final int[] CLIMBER_INVERTED_MOTOR_PORTS = {};
    public static final TypeOfMotor[] CLIMBER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };
    public static final boolean CLIMBER_ENCODER_IS_INVERTED = false;
}
