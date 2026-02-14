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

    public int getIntakedFuel() {
        return roller.getIntakedFuel();
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
        return runOnce(() -> extender.extend())
                .until(extender.isExtended())
                .andThen(runEnd(() -> roller.start(), () -> roller.stop()));
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> roller.stop()).andThen(runOnce(() -> stowIntake()));
    }

    public Command goToSiftAngleOneCommand() {
        return runOnce(() -> extender.goToSiftAngleOne());
    }

    public Command goToSiftAngleTwoCommand() {
        return runOnce(() -> extender.goToSiftAngleTwo());
    }

    public Command outtakeCommand() {
        return runOnce(() -> extender.extend()).andThen(runEnd(() -> roller.outtake(), () -> roller.stop()));
    }

    public Command siftFuelCommand() {
        return run(() -> Commands.repeatingSequence(
                        runOnce(() -> extender.goToSiftAngleOne()).until(extender.atTarget()),
                        runOnce(() -> extender.goToSiftAngleTwo()).until(extender.atTarget())))
                .andThen(() -> extender.extend());
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerInputs);
        extender.updateInputs(extenderInputs);
        extender.periodic();
        Logger.processInputs("Intake/Extender", extenderInputsAutoLogged);
    }
}
