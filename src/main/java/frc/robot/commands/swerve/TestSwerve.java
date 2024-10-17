// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.ArrayList;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.HASwerveModule;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class TestSwerve extends Command {
  SwerveDriveTrain swerveDriveTrain;
  ArrayList<HASwerveModule> modules;
  double mps;

  /** Creates a new TestSwerve. */
  public TestSwerve(double mps) {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    this.mps = mps;
    modules = swerveDriveTrain.getModules();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
for (HASwerveModule swerveModule : modules) {
      swerveModule.controlSwerveSpeed(mps);
      // swerveModule.controlSwerveDirection(0.5);
    }  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if(swerveDriveTrain.{
     * return true;
     * } else {
     */
    return false;
  }
}
