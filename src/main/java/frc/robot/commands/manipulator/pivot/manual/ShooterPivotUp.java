// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.pivot.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.ShooterPivot;

public class ShooterPivotUp extends Command {
  ShooterPivot shooterPivot;

  /** Creates a new ShooterPivotDown. */
  public ShooterPivotUp() {
    shooterPivot = Robot.getRobotContainer().getShooterPivot();

    addRequirements(shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPivot.driveShooterPivot(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPivot.driveShooterPivot(0);
    //shooterPivot.setCurrentTarget(shooterPivot.getShooterPivotPositionInDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
