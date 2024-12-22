/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class ProfiledDriveStraight extends Command {
  private double a;
  private double v;
  private double t;
  private double distance;

  private SwerveSubsystem drivetrain;
  private final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          SwerveSubsystem.getMaxSpeed(), SwerveSubsystem.getMaxAcceleration());
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(CONSTRAINTS);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private TrapezoidProfile.State currState = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private Timer timer = new Timer();

  /** Creates a new ProfiledDriveStraight. */
  public ProfiledDriveStraight(Subsystems subsystems, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = subsystems.drivetrain;
    this.distance = distance;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    currState.position = drivetrain.getPosition().getX();
    currState.velocity = drivetrain.getChassisSpeeds().vxMetersPerSecond;

    goalState.position = currState.position + distance;
    goalState.velocity = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currState.position = drivetrain.getPosition().getX();
    currState.velocity = drivetrain.getChassisSpeeds().vxMetersPerSecond;

    chassisSpeeds.vxMetersPerSecond =
        trapezoidProfile.calculate(timer.get(), currState, goalState).velocity;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isFinished()) {
      chassisSpeeds.vxMetersPerSecond = 0;
      drivetrain.setChassisSpeeds(chassisSpeeds);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapezoidProfile.isFinished(timer.get());
  }
}
