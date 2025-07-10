/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autonomous.Auto;
import frc.robot.Command.coralAutoL1;
import frc.robot.Command.coralAutoL24;
//import frc.robot.Command.sensorCoral;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  private final coralAutoL1 m_coralAutoL1 = new coralAutoL1(m_coralSubsystem);
  private final coralAutoL24 m_coralAutoL24 = new coralAutoL24(m_coralSubsystem);

  private final Auto m_auto = new Auto(m_elevatorSubsystem, m_coralAutoL24, m_coralAutoL1);

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController2 = new CommandXboxController(OperatorConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

     SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                        () -> m_driverController.getLeftY() * 0.65,
                                                        () -> m_driverController.getLeftX() * 0.65)
                                                        .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                          4)*-0.8)
                                                      .deadband(OperatorConstants.deadband)
                                                      .scaleTranslation(0.7) //value for drive speed
                                                      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  m_driverController::getRightY).headingWhile(true);
  
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2)*0.5)
                                                                    .deadband(OperatorConstants.deadband)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(false);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);
  
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    /*named commands for pathplanner */
    /*NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("ElevatorL4",  new ElevatorSubsystem().setElevatorHeight(.635) );
    
    NamedCommands.registerCommand("ElevatorL3",  new ElevatorSubsystem().setElevatorHeight(0.315) );
    NamedCommands.registerCommand("ElevatortSow",  new ElevatorSubsystem().setElevatorHeight(-0.05) );
    NamedCommands.registerCommand("IntakeCoral", new IntakeCommand(m_coralSubsystem));
    NamedCommands.registerCommand("ReverseCoral", new ReverseCommand(m_coralSubsystem));
    NamedCommands.registerCommand("GrabAlgae", new GrabAlgaeCommand(m_algaeSubsystem));
    NamedCommands.registerCommand("StopAlgae", new StopAlgaeCommand(m_algaeSubsystem));
    NamedCommands.registerCommand("StowAlgae", new StowCommand(m_algaeSubsystem));
     */
    NamedCommands.registerCommand("ScoreL1", m_auto.autoL1());
    NamedCommands.registerCommand("ScoreL2", m_auto.autoL2());
    //NamedCommands.registerCommand("StopCoral", m_coralSubsystem.coralStop());
    //NamedCommands.registerCommand("ElevatorL2",  new ElevatorSubsystem().setElevatorHeight(0.11) );
    //NamedCommands.registerCommand("Score high", new ScoreCoralCommand(m_coralSubsystem, true));
    

    CanBridge.runTCP();


    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.setElevatorHeight(-0.05));
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.setPower(0));
    m_coralSubsystem.setDefaultCommand(m_coralSubsystem.coralStop());
    m_algaeIntakeSubsystem.setDefaultCommand(m_algaeIntakeSubsystem.stopIntake());


    configureBindings();

    //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  // Trigger & Button Bindings!
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveRobotOrientedAngularVelocity = drivebase.drive(driveAngularVelocity);
    SmartDashboard.putData("Side View", Constants.sideView);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

    //new Trigger(() -> m_coralSubsystem.isHoldingCoralViaLaserCAN())
    //.onTrue(new sensorCoral(m_coralSubsystem))
    //.onFalse(m_coralSubsystem.coralStop());
 
  m_driverController2.rightTrigger().whileTrue(m_algaeSubsystem.setAlgaeArmAngle(63));  // algae pivot setpoints
  m_driverController2.rightTrigger().whileFalse(m_algaeSubsystem.setAlgaeArmAngle(105));  // algae pivot setpoints

  m_driverController2.leftTrigger().whileTrue(m_algaeSubsystem.setAlgaeArmAngle(2));
  m_driverController2.leftTrigger().whileFalse(m_algaeSubsystem.setAlgaeArmAngle(105));

  m_driverController2.button(1).whileTrue(m_coralSubsystem.coralL1()); 
  m_driverController2.button(1).whileFalse(m_coralSubsystem.coralStop());

  m_driverController2.button(5).whileTrue(m_coralSubsystem.coralL24()); 
  m_driverController2.button(5).whileFalse(m_coralSubsystem.coralStop());  
  
  m_driverController2.button(6).whileTrue(m_coralSubsystem.coralL24LowSpeed());
  m_driverController2.button(6).whileFalse(m_coralSubsystem.coralStop());
  
  m_driverController2.button(3).whileTrue(m_coralSubsystem.coralReverse());
  m_driverController2.button(3).whileFalse(m_coralSubsystem.coralStop());

  m_driverController2.button(2).whileTrue(m_elevatorSubsystem.setElevatorHeight(0.11).repeatedly());
  m_driverController2.button(4).whileTrue(m_elevatorSubsystem.setElevatorHeight(0.315).repeatedly());
  m_driverController.button(3).whileTrue(m_elevatorSubsystem.setElevatorHeight(.635).repeatedly());
  /* 
  m_driverController2.rightTrigger(0.34).whileTrue(m_algaeSubsystem.moveDown(0.24));
  m_driverController2.rightTrigger(0.67).whileTrue(m_algaeSubsystem.moveDown(0.47));
  m_driverController2.rightTrigger(1).whileTrue(m_algaeSubsystem.moveDown(0.7));
  m_driverController2.rightTrigger().whileFalse(m_algaeSubsystem.moveStop());
  
  m_driverController2.leftTrigger(0.34).whileTrue(m_algaeSubsystem.moveUp(-0.24));
  m_driverController2.leftTrigger(0.67).whileTrue(m_algaeSubsystem.moveUp(-0.47));
  m_driverController2.leftTrigger(1).whileTrue(m_algaeSubsystem.moveUp(-0.7));
  m_driverController2.leftTrigger().whileFalse(m_algaeSubsystem.moveStop());
*/
  m_driverController2.button(10).whileTrue(m_algaeIntakeSubsystem.takeAlgae());
  m_driverController2.button(10).whileFalse(m_algaeIntakeSubsystem.stopIntake());

  m_driverController2.button(9).whileTrue(m_algaeIntakeSubsystem.scoreAlgae());
  m_driverController2.button(9).whileFalse(m_algaeIntakeSubsystem.stopIntake());

  m_driverController.button(1).whileTrue(m_elevatorSubsystem.resetEncoder());

  //m_driverController2.button(0).whileTrue(m_elevatorSubsystem.setElevatorHeight(-0.05));
  //m_driverController.button(1).whileTrue(driveFieldOrientedDirectAngleKeyboard);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


 public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("station2");
  }
}
