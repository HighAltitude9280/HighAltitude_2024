/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources.components.speedController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Add your docs here.
 */
public class HighAltitudeMotorGroup {
    private List<HighAltitudeMotorController> motors;
    private HashMap<Integer, HighAltitudeMotorController> motorsHashMap;

    private HighAltitudeMotorController encodedMotor;
    private boolean encoderIsInverted;

    /**
     * Creates a new list of {@link HighAltitudeMotorController}.
     * With this class you can control a whole set of motors by just
     * calling one method.
     * It can also be used to access a motor in a specific port in that list.
     *
     * @param ports              Requires an array of ports
     * @param invertedMotorPorts Requires an array that indicates inverted motor
     *                           ports
     * @param motorTypes         Requires an array of {@link TypeOfMotor}
     */
    public HighAltitudeMotorGroup(int[] ports, int[] invertedMotorPorts, TypeOfMotor[] motorTypes) {
        if (ports.length != motorTypes.length) {
            DriverStation.reportError("Amount of motor types and motor ports is not equal.", true);
            return;
        }
        motorsHashMap = new HashMap<>();
        motors = new ArrayList<>();
        for (int i = 0; i < ports.length; i++) {
            motors.add(new HighAltitudeMotorController(ports[i], motorTypes[i]));
            for (int invertedPort : invertedMotorPorts) {
                if (invertedPort == ports[i])
                    motors.get(i).setInverted(true);
            }
            motorsHashMap.put(ports[i], motors.get(i));
        }
        encodedMotor = motors.get(0);
        encoderIsInverted = false;

    }

    /**
     * Creates a new list of {@link HighAltitudeMotorController}.
     * With this class you can control a whole set of motors by just
     * calling one method.
     * It can also be used to access a motor in a specific port in that list.
     *
     * @param controllers An array of {@link HighAltitudeMotorController}s
     */
    public HighAltitudeMotorGroup(HighAltitudeMotorController... controllers) {
        motorsHashMap = new HashMap<Integer, HighAltitudeMotorController>();
        motors = new ArrayList<HighAltitudeMotorController>();
        for (HighAltitudeMotorController controller : controllers) {
            motors.add(controller);
            motorsHashMap.put(controller.getPort(), controller);
        }
        encodedMotor = motors.get(0);
    }

    /**
     * Sets all motors in the list to desired speed.
     *
     * @param speed power from -1 to 1
     */
    public void setAll(double speed) {
        for (HighAltitudeMotorController motor : motors) {
            motor.set(speed);
        }
    }

    public void setVoltage(double volts) {
        for (HighAltitudeMotorController motor : motors) {
            motor.setVoltage(volts);
        }
    }

    /**
     * Sets a specific motor to a given speed.
     *
     * @param port  motor port
     * @param speed speed ranging from -1 to 1
     */
    public void setSpecificMotorSpeed(int port, double speed) {
        motorsHashMap.get(port).set(speed);
    }

    /**
     * This can be used to get a specific motor, mainly used to get the encoder.
     *
     * @param port motor port
     * @return {@link HighAltitudeMotorController}
     */
    public HighAltitudeMotorController getSpecificMotor(int port) {
        return motorsHashMap.get(port);
    }

    public void setInverted(boolean inverted) {
        for (HighAltitudeMotorController motor : motors) {
            motor.setInverted(inverted);
        }
    }

    public int getSize() {
        return motors.size();
    }

    public List<HighAltitudeMotorController> getMotors() {
        return motors;
    }

    public void setEncoder(int port) {
        encodedMotor = motorsHashMap.get(port);
    }

    public void setEncoderInverted(boolean isInverted) {
        encoderIsInverted = isInverted;
    }

    public double getEncoderPosition() {
        return (encoderIsInverted ? -1 : 1) * encodedMotor.getEncPosition();
    }

    public double getEncoderVelocity() {
        return (encoderIsInverted ? -1 : 1) * encodedMotor.getEncVelocity();
    }

    public void setBrakeMode(boolean doBrake) {

        for (HighAltitudeMotorController motor : motors) {
            motor.setBrakeMode(doBrake);
        }

    }

    public void resetEncoder() {
        encodedMotor.setEncoderPosition(0);
    }
}