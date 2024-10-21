// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ControlShooter extends Command {

  int rpm;

  /** Creates a new ControlShooter. */
  public ControlShooter(int rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rpm = rpm;
    addRequirements(Robot.getRobotContainer().getShooter());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean onTarget = Robot.getRobotContainer().getShooter().controlShooter(rpm);
    if (onTarget)
      OI.getInstance().getPilot().getHaptics().rumble(1, 0.1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getShooter().stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
