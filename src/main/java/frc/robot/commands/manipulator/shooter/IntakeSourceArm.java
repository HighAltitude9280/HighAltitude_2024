// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.manipulator.pivot.ShooterArmMaintainTo;
import frc.robot.commands.manipulator.transition.IntakeSource;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSourceArm extends ParallelCommandGroup {
  /** Creates a new IntakeSourceArm. */
  public IntakeSourceArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterArmMaintainTo(107.5, 0.2),
        new SequentialCommandGroup(
            new WaitCommand(1.5),
            new IntakeSource()));
  }
}
