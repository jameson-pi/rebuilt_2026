package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private RollerIO roller;
    private ExtenderIO extender;
    private RollerIO.IntakeIOInputs rollerInputs;
    private ExtenderIO.ExtenderIOInputs extenderInputs;
    private ExtenderIOInputsAutoLogged extenderInputsAutoLogged;

    public Intake(RollerIO rollerIO, ExtenderIO extenderIO) {
        roller = rollerIO;
        extender = extenderIO;
        rollerInputs = new RollerIO.IntakeIOInputs();
    }

    // Extender Commands
    public Command extendIntake() {
        return runOnce(() -> extender.extend());
    }

    public Command stowIntake() {
        return runOnce(() -> extender.retract());
    }

    public Command toggleIntake() {
        return runOnce(() -> extender.toggle());
    }

    // Roller Commands
    public Command intakeRollerCommand() {
        return Commands.startEnd(() -> roller.start(), () -> roller.stop());
    }

    public Command outtakeRollerCommand() {
        return Commands.startEnd(() -> roller.outtake(), () -> roller.stop());
    }

    public Command stopRollerCommand() {
        return runOnce(() -> roller.stop());
    }

    public Command intakeCommand() {
        return runOnce(() -> extendIntake()).andThen(runEnd(() -> intakeRollerCommand(), () -> stopRollerCommand()));
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> stopRollerCommand()).andThen(runOnce(() -> stowIntake()));
    }

    public Command goToSiftAngleOneCommand() {
        return runOnce(() -> extender.goToSiftAngleOne());
    }

    public Command goToSiftAngleTwoCommand() {
        return runOnce(() -> extender.goToSiftAngleTwo());
    }

    public Command outtakeCommand() {
        return runOnce(() -> extendIntake()).andThen(runEnd(() -> outtakeRollerCommand(), () -> stopRollerCommand()));
    }

    public Command siftFuelCommand() {
        return run(() -> Commands.repeatingSequence(goToSiftAngleOneCommand(), goToSiftAngleTwoCommand()))
                .andThen(extendIntake());
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerInputs);
        extender.updateInputs(extenderInputs);
        extender.periodic();
        Logger.processInputs("Intake/Extender", extenderInputsAutoLogged);
    }
}
