package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final RollerIO roller;
    private final ExtenderIO extender;
    private final RollerIO.RollerIOInputs rollerInputs;
    private final ExtenderIO.ExtenderIOInputs extenderInputs;

    public Intake(RollerIO rollerIO, ExtenderIO extenderIO) {
        roller = rollerIO;
        extender = extenderIO;
        rollerInputs = new RollerIOInputsAutoLogged();
        extenderInputs = new ExtenderIOInputsAutoLogged();
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
        // return new ConditionalCommand(
        // runOnce(() -> extender.retract()), runOnce(() -> extender.extend()),
        // extender.isExtended());
        return runOnce(() -> extender.toggle());
    }

    // Roller Commands
    public Command intakeRollerCommand() {
        return Commands.runEnd(() -> roller.start(), () -> roller.stop(), this);
    }

    public Command outtakeRollerCommand() {
        return Commands.runEnd(() -> roller.outtake(), () -> roller.stop(), this);
    }

    public Command stopRollerCommand() {
        return runOnce(() -> roller.stop());
    }

    // Combination Commands
    public Command intakeCommand() {
        return runOnce(() -> extender.extend())
                .until(extender.isExtended())
                .andThen(runEnd(() -> roller.start(), () -> roller.stop()));
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> roller.stop()).andThen(runOnce(() -> extender.retract()));
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

    // Utility Commands
    public Command zeroExtender() {
        return runOnce(extender::zero);
    }

    public boolean isRollerRunning() {
        return Math.abs(rollerInputs.rollerSpeedPercentile) > 0.1;
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerInputs);
        extender.updateInputs(extenderInputs);
        extender.periodic();
        roller.periodic();
        Logger.recordOutput(
                "Intake/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
    }
}
