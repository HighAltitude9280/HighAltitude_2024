// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip.commands.primitives;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetRGB extends Command {
  int r, g, b;

  /** Creates a new SetRGB. */
  public SetRGB(int r, int g, int b) {
    addRequirements(Robot.getRobotContainer().getLEDs());
    this.r = r;
    this.g = g;
    this.b = b;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getRobotContainer().getLEDs().setRGB(r, g, b);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getLEDs().allLedsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
