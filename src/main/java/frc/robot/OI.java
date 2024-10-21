// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.commands.swerve.TestSwerve;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

/** Add your docs here. */
public class OI {
    public static OI instance;

    private HighAltitudeJoystick pilot;
    private HighAltitudeJoystick copilot;

    public void ConfigureButtonBindings() {
        ////////////////////////// PILOT //////////////////////////

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case Joakin:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));

                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START, ButtonType.BACK);
                pilot.onTrue(ButtonType.X, new DefaultSwerveDriveNew());

                pilot.onTrue(ButtonType.RT, new TestSwerve(0.5));
                pilot.onTrue(ButtonType.LT, new TestSwerve(-0.5));

                
                /*
                 * pilot.whileTrue(ButtonType.X, new DriveIntake(-0.1));
                 * 
                 * pilot.whileTrue(ButtonType.POV_S, new IntakePivotMoveTo(0.5, 0));
                 * pilot.onTrue(ButtonType.POV_N, new ShooterPivotSetAngleTarget(20));
                 * pilot.whileTrue(ButtonType.POV_N, new ShooterPivotMaintainTarget(0.5));
                 * pilot.onTrue(ButtonType.POV_N, new IntakePivotRetractar(0.5));
                 * 
                 * pilot.onTrue(ButtonType.POV_S, new ShooterPivotSetAngleTarget(40));
                 * pilot.whileTrue(ButtonType.POV_S, new ShooterPivotMaintainTarget(0.5));
                 * pilot.onTrue(ButtonType.POV_S, new IntakePivotExtruir(0.5));
                 * // ARREGLAR ESTO
                 * 
                 * 
                 * pilot.whileTrue(ButtonType.RB, new IntakeAndRollersOut());
                 * pilot.whileTrue(ButtonType.LB, new ShooterIntake());
                 * pilot.whileTrue(ButtonType.LB, new IntakeOut());
                 * pilot.whileTrue(ButtonType.LB, new RollersInUntilNoNote());
                 * 
                 * pilot.whileTrue(ButtonType.RT, new ControlShooter(4000));
                 * pilot.whileTrue(ButtonType.LT, new IntakeIn());
                 * 
                 * // pilot.whileTrue(ButtonType.A, new followTarget());
                 * 
                 * pilot.whileTrue(ButtonType.POV_SE, new ToggleIsOnCompetitiveField());
                 * 
                 * // pilot.toggleOnTrue(ButtonType.A, new SwerveDriveAndCenter());
                 * // pilot.onTrue(ButtonType.A, new ShooterPivotSetAngleTarget(22));
                 * // pilot.whileTrue(ButtonType.A, new ShooterPivotMaintainTarget(0.5));
                 * pilot.whileTrue(ButtonType.POV_E, new
                 * ShooterPivotMaintainTargetAndRollers(0.5, 12));
                 * 
                 * pilot.whileTrue(ButtonType.POV_W, new
                 * ShooterPivotMaintainTargetAndRollers(0.5, 35));
                 * 
                 * pilot.onTrue(ButtonType.A, new IntakeAutoTransition());
                 * pilot.whileTrue(ButtonType.A, new DriveToClosestNote(
                 * HighAltitudeConstants.NOTE_DETECTION_TURN_POWER,
                 * HighAltitudeConstants.NOTE_DETECTION_DRIVE_POWER));
                 * 
                 * pilot.onTrue(ButtonType.B, new AutoAmp());
                 * pilot.whileTrue(ButtonType.B, new ShooterPivotMaintainTarget(-30, 0.5));
                 * pilot.whileTrue(ButtonType.B, new IntakeOut());
                 */
                break;

        }
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case DefaultUser:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                break;
        }

    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultSwerveDriveSpeed() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return -pilot.getAxis(AxisType.LEFT_Y);

            case Joakin:
                return -pilot.getAxis(AxisType.LEFT_Y);

            default:
                return -pilot.getAxis(AxisType.LEFT_Y);

        }
    }

    public double getDefaultSwerveDriveStrafe() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return -pilot.getAxis(AxisType.LEFT_X);

            case Joakin:
                return -pilot.getAxis(AxisType.LEFT_X);

            default:
                return -pilot.getAxis(AxisType.LEFT_X);
        }
    }

    public double getDefaultSwerveDriveTurn() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return -pilot.getAxis(AxisType.RIGHT_X);

            case Joakin:
                return -pilot.getAxis(AxisType.RIGHT_X);

            default:
                return -pilot.getAxis(AxisType.RIGHT_X);
        }
    }

    public double getDeafultShooterDriveSpeed() {
        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getTriggers();

            case Joakin:
                return pilot.getTriggers();

            default:
                return pilot.getTriggers();
        }
    }

    public HighAltitudeJoystick getPilot() {
        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot;

            case Joakin:
                return pilot;

            default:
                return pilot;
        }
    }

    public HighAltitudeJoystick getCopilot() {
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case DefaultUser:
                return copilot;

            case Joakin:
                return copilot;

            default:
                return copilot;
        }
    }

}
