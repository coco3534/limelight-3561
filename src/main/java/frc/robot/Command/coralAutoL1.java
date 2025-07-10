// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralAutoL1 extends Command {
  /** Creates a new coralAutoL1. */
  private final CoralSubsystem coralSubsystem;
  public coralAutoL1(CoralSubsystem coralSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralSubsystem);
    this.coralSubsystem = coralSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralSubsystem.scoreAutoL1();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSubsystem.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
