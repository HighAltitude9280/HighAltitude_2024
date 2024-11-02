// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IndexerUntilNote extends Command {
  /** Creates a new IntakeUntilNote. */
  public IndexerUntilNote() {
    // Use addRequirements() here to declare subsystem dependencies.}
    addRequirements(Robot.getRobotContainer().getIndexer());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getRobotContainer().getIndexer().indexerOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getIndexer().stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Robot.getRobotContainer().getIndexer().hasNote();
    return false;
  }
}