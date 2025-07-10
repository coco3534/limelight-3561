// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Command.coralAutoL1;
import frc.robot.Command.coralAutoL24;
import frc.robot.subsystems.ElevatorSubsystem;

//file for autonomous commands
public class Auto{
    //define whatever file that you are gonna use the method from
    private ElevatorSubsystem elevatorSubsystem;
    private coralAutoL24 coralAutoL24;
    private coralAutoL1 coralAutoL1;
     
    //connect subsystem, command, this, and robotContainer, adjust as more file is being used
    public Auto(ElevatorSubsystem elevator, coralAutoL24 coralL24, coralAutoL1 coralL1){
        elevatorSubsystem = elevator;
        coralAutoL24 = coralL24;
        coralAutoL1 = coralL1;
    }

    //L1 score auto command
    public Command autoL1(){
        return coralAutoL1.withTimeout(1);
    }

    //L2 score auto command
    public Command autoL2(){
        return elevatorSubsystem.setElevatorHeight(0.11).andThen(elevatorSubsystem.hold())
        .andThen(coralAutoL24.withTimeout(1))
        .andThen(elevatorSubsystem.setElevatorHeight(-0.05));
    }
}
