// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntakeSubsystem. */
  //define your motor controller
private final SparkMax intakeMotor = new SparkMax(Constants.IDConstants.Algae_Intake_ID, MotorType.kBrushless);

//Algae Intake state constant storage
public enum IntakeState {
  NONE,
  INTAKE,
  OUTTAKE
}
private IntakeState mState = IntakeState.NONE;
//getStatus
  public IntakeState getState() {
    return mState;
  }

  //didn't use, but can put motor config here
  public AlgaeIntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  //Intake 
  public void algaeIntake() {
    mState = IntakeState.INTAKE;
    intakeMotor.set(Constants.Coral_Algae_Constants.kAlgaeIntakeSpeed);
  }
  public Command takeAlgae() {
    return run(() -> algaeIntake());
  }

  //Outtake
  public void algaeOuttake() {
    mState = IntakeState.OUTTAKE;
    intakeMotor.set(Constants.Coral_Algae_Constants.kAlgaeEjectSpeed);
  }
  public Command scoreAlgae() {
    return run(() -> algaeOuttake());
  }

  //stop 
  public void stopAlgaeIntake() {
    mState = IntakeState.NONE;
    intakeMotor.set(0);
  }
  public Command stopIntake() {
    return run(() -> stopAlgaeIntake());
  }
}
