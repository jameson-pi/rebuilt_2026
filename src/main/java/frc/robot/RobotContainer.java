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

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.state.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.OILayer.OI;
import frc.robot.util.OILayer.OIXbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Superstructure superstructure;

    // OI Layer
    private final OI oi = new OIXbox();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState.create();

        /*
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                vision = new Vision(drive); // Add vision IOs as needed
                break;
            case SIM:
                driveSimulation =
                        new SwerveDriveSimulation(Drive.createMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        (pose) -> driveSimulation.setSimulationWorldPose(pose));
                vision = new Vision(drive);
                break;
            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive);
                break;
        }
        */

        superstructure = new Superstructure();
        /*
        if (Constants.currentMode == Constants.Mode.SIM) {
            superstructure.configureGamePieceSimulation(driveSimulation);
        }
        */

        // Configure the button bindings
        configureButtonBindings();

        // robotState.setPoseSupplier(drive::getPose);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -oi.driveTranslationY().getAsDouble(),
                () -> -oi.driveTranslationX().getAsDouble(),
                () -> -oi.driveRotation().getAsDouble()));

        // Lock to 0° when button is held
        oi.driveLock0()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -oi.driveTranslationY().getAsDouble(),
                        () -> -oi.driveTranslationX().getAsDouble(),
                        () -> new Rotation2d()));

        // Full auto-aim (aims robot + sets shooter RPM and hood angle)
        Command autoAimCommand = superstructure.fullAutoAim(
                drive, () -> -oi.driveTranslationY().getAsDouble(), () -> -oi.driveTranslationX()
                        .getAsDouble());

        if (Constants.currentMode == Constants.Mode.SIM) {
            autoAimCommand = autoAimCommand.alongWith(Commands.run(() -> {
                        if (superstructure.hasGamePieceTrajectorySimulation()) {
                            superstructure.getGamePieceTrajectorySimulation().previewTrajectory();
                        }
                    })
                    .withName("PreviewTrajectory"));
        }

        oi.spinUpShooter().whileTrue(autoAimCommand);

        // Manual fire (feeds piece when shooter is ready)
        oi.fireShooter().whileTrue(superstructure.fireCommand()).onFalse(superstructure.stopUpgoerCommand());

        // Stop all components
        oi.stopSuperstructure()
                .onTrue(superstructure.stopShooterCommand().alongWith(superstructure.stopUpgoerCommand()));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        oi.zeroDrivebase().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        */

        // Shooter Bring-up / Bench Mode Bindings
        oi.spinUpShooter().whileTrue(superstructure.autoSpeedShooter());

        oi.fireShooter().whileTrue(superstructure.fireCommand()).onFalse(superstructure.stopUpgoerCommand());

        oi.stopSuperstructure()
                .onTrue(superstructure.stopShooterCommand().alongWith(superstructure.stopUpgoerCommand()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void resetSimulation() {
        /*
        if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        */
    }

    public void updateSimulation() {
        /*
        if (driveSimulation != null) {
            Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        }
        */
    }
}
