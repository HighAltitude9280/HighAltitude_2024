// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterPivotSetAngleTarget extends InstantCommand {
  double target;

  public ShooterPivotSetAngleTarget(double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getShooterPivot());
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getRobotContainer().getShooterPivot().setAngleTarget(this.target);
  }
}
