// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class RobotMath {

   public static class AlgaeArm
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertAlgaeAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * AlgaeArmConstants.kAlgaeArmReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAlgaeAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / AlgaeArmConstants.kAlgaeArmReduction);

    }
  }
    public static class Elevator {
        public static Angle convertDistanceToRotations(Distance distance)
    {
      // m/(2*pi*r)*g = e
      return Rotations.of((distance.in(Meters) /
                          (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)) *
                          ElevatorConstants.kElevatorGearing);
    }

    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI));
    }
    }
}
