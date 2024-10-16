// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.resources.components.Navx;
import frc.robot.resources.components.PWMLEDStrip.LEDs;
import frc.robot.resources.components.PWMLEDStrip.commands.primitives.SetRGB;
//import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.vision.vision;

/** Add your docs here. */
public class RobotContainer {
    enum Mode {
        MANUAL,
        NOTE,
        AMPLIFIED,
        COOPERTITION
    }

    private Mode currentMode = Mode.MANUAL;
    private Navx navx;
    private SwerveDriveTrain swerveDriveTrain;
    private vision vision;
    private LEDs leds;
    private boolean isOnField;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        navx = new Navx();
        leds = new LEDs();
        swerveDriveTrain = new SwerveDriveTrain();
        vision = new vision();
    }

    public void ConfigureButtonBindings() {
        OI.getInstance().ConfigureButtonBindings();
        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case Joakin:
                break;

            default:
                break;
        }

        switch (HighAltitudeConstants.CURRENT_COPILOT) {
            default:
                break;
        }

        swerveDriveTrain.setDefaultCommand(new DefaultSwerveDriveNew());
        leds.setDefaultCommand(new SetRGB(0, 255, 137));
        // climber.setDefaultCommand(new MaintainClimberPosition());
    }

    public Navx getNavx() {
        return navx;
    }

    public LEDs getLEDs() {
        return leds;
    }

    ////////////////////////////////////////////////

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void putAutoChooser() {
        SmartDashboard.putData("Autonomous", m_chooser);
    }

    public boolean getIsOnField() {
        return isOnField;
    }

    public void toggleIsOnField() {
        isOnField = !isOnField;
    }

    public Mode getCurrentMode() {
        return currentMode;
    }

    public void setCurrentMode(Mode mode) {
        currentMode = mode;
    }

    public SwerveDriveTrain getSwerveDriveTrain() {
        return swerveDriveTrain;
    }

    public vision getVision() {
        return vision;
    }

}
