// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.vision.vision;

public class followTarget extends Command {

  vision vision;

  /** Creates a new followTarget. */
  public followTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    // addRequirements(Robot.getRobotContainer().getShooterPivot());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision = Robot.getRobotContainer().getVision();
    Robot.getRobotContainer().getSwerveDriveTrain().setIsFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasNoteTargets()) {
      double yaw = vision.getYaw();
      double area = vision.getArea();
      // Robot.getRobotContainer().getSwerveDriveTrain().followTarget(yaw, area, 1.53,
      // 0.2);
      // Robot.getRobotContainer().getShooterPivot().followTarget();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().setIsFieldOriented(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
