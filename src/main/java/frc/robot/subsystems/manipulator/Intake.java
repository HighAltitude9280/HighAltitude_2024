// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Intake extends SubsystemBase {
  HighAltitudeMotorGroup intakeMotors;
  ColorSensorV3 intakeColorSensor;
  AnalogInput proximitySensor;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotors = new HighAltitudeMotorGroup(RobotMap.INTAKE_MOTOR_PORTS, RobotMap.INTAKE_INVERTED_MOTORS_PORTS,
        RobotMap.INTAKE_MOTOR_TYPES);

    intakeColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    proximitySensor = new AnalogInput(RobotMap.INTAKE_ANALOG_SENSOR_PORT);
  }

  public void driveIntake(double speed) {
    intakeMotors.setAll(speed);
  }

  public void intakeIn() {
    intakeMotors.setAll(HighAltitudeConstants.INTAKE_IN_SPEED);
  }

  public void intakeOut() {
    intakeMotors.setAll(HighAltitudeConstants.INTAKE_OUT_SPEED);
  }


  public void stopIntake() {
    intakeMotors.setAll(0);
  }

  public RawColor detectedColor() {
    return intakeColorSensor.getRawColor();
  }

  public double getDetectedColorRed() {
    return intakeColorSensor.getRawColor().red;
  }

  public int getProximity() {
    return proximitySensor.getAverageValue();
  }

  public boolean hasNote() {
    return intakeColorSensor.getRawColor().red >= 500;
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("IntakeHasNote", hasNote());
    SmartDashboard.putNumber("Detected red", intakeColorSensor.getRawColor().red);
    // This method will be called once per scheduler run
  }
}