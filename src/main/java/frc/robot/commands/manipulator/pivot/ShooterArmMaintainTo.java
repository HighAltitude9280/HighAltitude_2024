// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterArmMaintainTo extends Command {
  /** Creates a new ShooterPivotMaintainTarget. */

  double maxPower;
  double target;

  public ShooterArmMaintainTo(double maxPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getShooterPivot());

    this.target = Double.NaN;
    this.maxPower = maxPower;
  }

  public ShooterArmMaintainTo(double angleTarget, double maxPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getShooterPivot());
    this.maxPower = maxPower;
    this.target = angleTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Double.isNaN(target))
      Robot.getRobotContainer().getShooterPivot().setAngleTarget(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getRobotContainer().getShooterPivot().maintainTarget(maxPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getShooterPivot().driveShooterPivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
