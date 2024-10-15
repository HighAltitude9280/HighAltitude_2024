// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.resources.components.PWMLEDStrip.commands.primitives.SetRGBThenTurnOff;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlashColor extends SequentialCommandGroup {
  /** Creates a new FlashColor. */
  public FlashColor(int r, int g, int b, double timeout) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetRGBThenTurnOff(r, g, b).withTimeout(timeout),
        new WaitCommand(timeout),
        new SetRGBThenTurnOff(r, g, b).withTimeout(timeout),
        new WaitCommand(timeout),
        new SetRGBThenTurnOff(r, g, b).withTimeout(timeout),
        new WaitCommand(timeout));
  }
}
