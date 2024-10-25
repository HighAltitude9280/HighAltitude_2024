// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.primitiveAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoAim extends Command {
  /** Creates a new MaintainPointAtTarget. */
  double shooterMaxPower, turnMaxPower;

  public AutoAim(double shooterMaxPower, double turnMaxPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    addRequirements(Robot.getRobotContainer().getShooterPivot());

    this.shooterMaxPower = shooterMaxPower;
    this.turnMaxPower = turnMaxPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot.getRobotContainer().getShooterPivot().pointToSpeaker(shooterMaxPower);
    Robot.getRobotContainer().getSwerveDriveTrain().pointToSpeaker(turnMaxPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
