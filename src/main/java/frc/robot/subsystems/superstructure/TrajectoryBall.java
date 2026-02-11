// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Utility class for calculating shooter trajectories, including Shooting on the Fly (SotF). */
public class TrajectoryBall {

    /** Resulting setpoints for a shooting maneuver. */
    public record ShootingParameters(Angle hoodAngle, AngularVelocity flywheelVelocity, Rotation2d targetHeading) {}

    /** Internal record for trajectory flight stats. */
    private record TrajectoryResult(Angle launchAngle, LinearVelocity launchSpeed, double totalTime) {}

    /**
     * Calculates shooting parameters for a stationary or moving robot.
     *
     * @param hasHood Whether the robot has an active hood subsystem
     * @param robotPose Current field pose
     * @param robotSpeeds Current field-relative or robot-relative speeds (for SotF)
     * @param maxHeight Calculated peak height of the trajectory (ignored if no hood)
     * @param targetHeight Height of the target opening
     * @param hoodAngleOffset User-defined fine-tuning offset
     * @param rpmMultiplier User-defined fine-tuning multiplier
     * @param isSotfEnabled Whether to apply movement compensation
     * @return Calculated setpoints
     */
    public static ShootingParameters calculate(
            boolean hasHood,
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            Distance maxHeight,
            Distance targetHeight,
            double hoodAngleOffset,
            double rpmMultiplier,
            boolean isSotfEnabled) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        Translation2d robotPosition = robotPose.getTranslation();
        Distance staticDistance = Meters.of(robotPosition.getDistance(hubPosition));
        Rotation2d staticAngle =
                new Rotation2d(hubPosition.getX() - robotPosition.getX(), hubPosition.getY() - robotPosition.getY());

        // 1. Initial stationary trajectory
        TrajectoryResult stationary;
        if (hasHood) {
            stationary = calculateStationary(staticDistance, maxHeight, targetHeight);
        } else {
            stationary = calculateFixedAngle(staticDistance, targetHeight, ShooterConstants.fixedHoodAngle);
        }

        double t = stationary.totalTime();

        if (!isSotfEnabled || robotSpeeds == null || t <= 0) {
            return finalizeParameters(
                    stationary.launchAngle, stationary.launchSpeed, staticAngle, hoodAngleOffset, rpmMultiplier);
        }

        // 2. Field-relative robot velocity
        Rotation2d heading = robotPose.getRotation();
        double fieldVx =
                robotSpeeds.vxMetersPerSecond * heading.getCos() - robotSpeeds.vyMetersPerSecond * heading.getSin();
        double fieldVy =
                robotSpeeds.vxMetersPerSecond * heading.getSin() + robotSpeeds.vyMetersPerSecond * heading.getCos();

        // 3. Compensation: v_ball_robot = v_ball_field - v_robot_field
        double desiredVfx = (hubPosition.getX() - robotPosition.getX()) / t;
        double desiredVfy = (hubPosition.getY() - robotPosition.getY()) / t;

        double launchVxField = desiredVfx - fieldVx;
        double launchVyField = desiredVfy - fieldVy;

        double horizontalLaunchSpeed = Math.sqrt(launchVxField * launchVxField + launchVyField * launchVyField);

        // 4. Vertical velocity
        double verticalVelocity;
        if (hasHood) {
            // Keep the same peak height
            double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
            double riseMeters = maxHeight.minus(ShooterConstants.shooterHeight).in(Meters);
            verticalVelocity = gravityMps2 * Math.sqrt(Math.max(0, 2.0 * riseMeters / gravityMps2));
        } else {
            // Fixed angle: v_z = v_x * tan(theta)
            verticalVelocity = horizontalLaunchSpeed * Math.tan(ShooterConstants.fixedHoodAngle.in(Radians));
        }

        Angle launchAngle = Radians.of(Math.atan2(verticalVelocity, horizontalLaunchSpeed));
        LinearVelocity totalLaunchSpeed = MetersPerSecond.of(
                Math.sqrt(horizontalLaunchSpeed * horizontalLaunchSpeed + verticalVelocity * verticalVelocity));
        Rotation2d targetHeading = new Rotation2d(launchVxField, launchVyField);

        return finalizeParameters(launchAngle, totalLaunchSpeed, targetHeading, hoodAngleOffset, rpmMultiplier);
    }

    private static TrajectoryResult calculateStationary(Distance distance, Distance maxHeight, Distance targetHeight) {
        Distance startHeight = ShooterConstants.shooterHeight;
        Distance riseHeight = maxHeight.minus(startHeight);
        Distance fallHeight = maxHeight.minus(targetHeight);

        if (riseHeight.in(Meters) <= 0 || fallHeight.in(Meters) < 0) {
            return new TrajectoryResult(Degrees.of(45.0), MetersPerSecond.of(0.0), 1.0);
        }

        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double riseMeters = riseHeight.in(Meters);
        double fallMeters = fallHeight.in(Meters);
        double distanceMeters = distance.in(Meters);

        double timeToRise = Math.sqrt(2.0 * riseMeters / gravityMps2);
        double timeToFall = Math.sqrt(2.0 * fallMeters / gravityMps2);
        double totalTime = timeToRise + timeToFall;

        double horizontalVelocity = distanceMeters / totalTime;
        double verticalVelocity = gravityMps2 * timeToRise;

        Angle launchAngle = Radians.of(Math.atan2(verticalVelocity, horizontalVelocity));
        LinearVelocity launchSpeed = MetersPerSecond.of(
                Math.sqrt(horizontalVelocity * horizontalVelocity + verticalVelocity * verticalVelocity));

        return new TrajectoryResult(launchAngle, launchSpeed, totalTime);
    }

    private static TrajectoryResult calculateFixedAngle(Distance distance, Distance targetHeight, Angle fixedAngle) {
        double d = distance.in(Meters);
        double theta = fixedAngle.in(Radians);
        double gravity = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double deltaH = targetHeight.minus(ShooterConstants.shooterHeight).in(Meters);

        // v = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaH)) )
        double cosTheta = Math.cos(theta);
        double denominator = 2 * cosTheta * cosTheta * (d * Math.tan(theta) - deltaH);

        if (denominator <= 0) {
            return new TrajectoryResult(fixedAngle, MetersPerSecond.of(0.0), 1.0);
        }

        double launchSpeedMps = Math.sqrt((gravity * d * d) / denominator);
        double timeOfFlight = d / (launchSpeedMps * cosTheta);

        return new TrajectoryResult(fixedAngle, MetersPerSecond.of(launchSpeedMps), timeOfFlight);
    }

    private static ShootingParameters finalizeParameters(
            Angle launchAngle, LinearVelocity launchSpeed, Rotation2d heading, double hoodOffset, double rpmMult) {
        // Apply offsets and clamps
        double hoodDegrees = launchAngle.in(Degrees) + hoodOffset;
        hoodDegrees = Math.max(
                ShooterConstants.minHoodAngle.in(Degrees),
                Math.min(ShooterConstants.maxHoodAngle.in(Degrees), hoodDegrees));

        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double angularVelRadPerSec =
                (launchSpeed.in(MetersPerSecond) / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        double rpm = RadiansPerSecond.of(angularVelRadPerSec).in(RPM) * rpmMult;

        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));

        return new ShootingParameters(Degrees.of(hoodDegrees), RPM.of(rpm), heading);
    }
}
