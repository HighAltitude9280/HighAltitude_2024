// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.manipulator.indexer.IndexerIn;
import frc.robot.commands.manipulator.indexer.IndexerOut;
import frc.robot.commands.manipulator.intake.IntakeIn;
import frc.robot.commands.manipulator.intake.IntakeOut;
import frc.robot.commands.manipulator.pivot.ShooterArmMaintainTo;
import frc.robot.commands.manipulator.pivot.ShooterPivotMaintainTarget;
import frc.robot.commands.manipulator.pivot.ShooterPivotSetAngleTarget;
import frc.robot.commands.manipulator.pivot.manual.ShooterPivotDown;
import frc.robot.commands.manipulator.pivot.manual.ShooterPivotUp;
import frc.robot.commands.manipulator.shooter.ControlShooter;
import frc.robot.commands.manipulator.shooter.DriveShooter;
import frc.robot.commands.manipulator.transition.IntakeIndexerIn;
import frc.robot.commands.manipulator.transition.IntakeIndexerOut;
import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.commands.swerve.SolisTestSwerve;
import frc.robot.commands.swerve.TestDirectionPIDSwerve;
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

                pilot.whileTrue(ButtonType.X, new TestDirectionPIDSwerve());

                // copilot.whileTrue(ButtonType.POV_E, new ShooterPivotUp());
                // copilot.whileTrue(ButtonType.POV_W, new ShooterPivotDown());

                // copilot.whileTrue(ButtonType.LT, new IntakeIndexerIn());
                // copilot.whileTrue(ButtonType.LB, new IndexerIn());

                // copilot.whileTrue(ButtonType.RT, new IntakeIndexerOut());
                // copilot.whileTrue(ButtonType.RB, new IndexerOut());

                // copilot.onTrue(ButtonType.A, new DriveShooter());

                // pilot.onTrue(ButtonType.LT, new ShooterArmMaintainTo(0, 0.1));
                // pilot.whileTrue(ButtonType.A, new ControlShooter(2000));

            case Yare:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));

                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START, ButtonType.BACK);

            default:
                break;

        }
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case Carlos:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.whileTrue(ButtonType.POV_E, new ShooterPivotUp());
                copilot.whileTrue(ButtonType.POV_W, new ShooterPivotDown());

                copilot.whileTrue(ButtonType.LT, new IntakeIndexerIn());
                copilot.whileTrue(ButtonType.LB, new IndexerIn());

                copilot.whileTrue(ButtonType.RT, new IntakeIndexerOut());
                copilot.whileTrue(ButtonType.RB, new IndexerOut());

                copilot.onTrue(ButtonType.A, new DriveShooter());

                // copilot.onTrue(ButtonType.LT, new ShooterArmMaintainTo(0, 0.1));
                // copilot.onTrue(ButtonType.RT, new ControlShooter(2000));

                break;
            default:
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
                return pilot.getAxis(AxisType.RIGHT_X);

            case Joakin:
                return pilot.getAxis(AxisType.RIGHT_X);

            default:
                return pilot.getAxis(AxisType.RIGHT_X);
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
