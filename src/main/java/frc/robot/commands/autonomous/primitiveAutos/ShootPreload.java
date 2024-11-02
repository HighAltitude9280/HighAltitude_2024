// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.primitiveAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.manipulator.pivot.ShooterPivotMaintainTarget;
import frc.robot.commands.manipulator.shooter.DriveShooter;
import frc.robot.commands.manipulator.transition.IntakeIndexerIn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPreload extends ParallelCommandGroup {
  /** Creates a new ShootPreload. */
  public ShootPreload() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new WaitCommand(1)),
        new ShooterPivotMaintainTarget(HighAltitudeConstants.SHOOTER_PIVOT_SPEAKER_SETPOINT,
            HighAltitudeConstants.SHOOTER_PIVOT_DRIVE_SPEED).withTimeout(5),
        new SequentialCommandGroup(
            new WaitCommand(1)),
        new DriveShooter().withTimeout(4),
        new SequentialCommandGroup(
            new WaitCommand(2),
            new IntakeIndexerIn().withTimeout(2.0)));
  }
}
