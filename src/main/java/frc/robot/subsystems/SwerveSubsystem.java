// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

public class SwerveSubsystem extends SubsystemBase {



File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive;
Limelight                limelight;
LimelightPoseEstimator   limelightPoseEstimator;
HolonomicDriveController alignmentController;

  public SwerveSubsystem() {

    

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
   
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setCosineCompensator(false);
    setupPathPlanner();
    setupLimelight();

  }

   public void setupLimelight()
  {
    swerveDrive.stopOdometryThread();
    limelight = new Limelight("limelight");
    limelight.getSettings()
             .withPipelineIndex(0)
             .withCameraOffset(new Pose3d(Units.inchesToMeters(12),
                                          Units.inchesToMeters(12),
                                          Units.inchesToMeters(10.5),
                                          new Rotation3d(0, 0, Units.degreesToRadians(45))))
             .withArilTagIdFilter(List.of(20.0, 21.0, 22.0, 9.0, 10.0, 11.0))
             .save();
    limelightPoseEstimator = limelight.getPoseEstimator(true);

  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() 
  {

    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(new Rotation3d(swerveDrive.getOdometryHeading()
                                                                               .rotateBy(Rotation2d.kZero)),
                                                     new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0))))
             .save();
    Optional<PoseEstimate>     poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results       = limelight.getLatestResults();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  

   public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
        swerveDrive::getPose,
        // Robot pose supplier
        swerveDrive::resetOdometry,
        // Method to reset odometry (will be called if your auto has a starting pose)
        swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

    public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
   /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */

  public SwerveDrive getSwerveDrive() {
   return swerveDrive;
  }
public void driveFieldOriented(ChassisSpeeds velocity) {
swerveDrive.driveFieldOriented(velocity);
}

public Command drive(Supplier<ChassisSpeeds> velocity){
  return run(() -> {swerveDrive.drive(velocity.get());
  });
}
public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
  return run(() -> {
    swerveDrive.driveFieldOriented(velocity.get());
    swerveDrive.getMaximumChassisAngularVelocity();
    swerveDrive.getMaximumChassisVelocity();
  });
}

public void zeroGyro(){
  swerveDrive.zeroGyro();
}

public Command initialDrive(){
  return run(() -> zeroGyro());
}
}