// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;

public class ShooterKiller extends Command 
{

  double lastSpeed;
  public ShooterKiller() 
  {
  }

  @Override
  public void initialize() 
  {
    lastSpeed = Robot.getRobotContainer().getShooter().getShooterLeftVel();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    lastSpeed = Robot.getRobotContainer().getShooter().getShooterLeftVel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    double currentSpeed = Robot.getRobotContainer().getShooter().getShooterLeftVel();
    boolean speedDrop = (currentSpeed - lastSpeed) < -Math.abs(HighAltitudeConstants.SHOOTER_VELOCITY_DROP_THRESHOLD);
    lastSpeed = currentSpeed;
    return speedDrop;
  }
}
