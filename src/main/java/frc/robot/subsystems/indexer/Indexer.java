package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity setpoint = RotationsPerSecond.of(0);

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/Setpoint", setpoint);
        Logger.recordOutput("Indexer/Running", Math.abs(setpoint.in(RotationsPerSecond)) > 0.1);
    }

    public Command index() {
        return runOnce(() -> {
            setpoint = IndexerConstants.kCollectorRPM;
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        });
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> indexerIO.setCustomSpeed(speed));
    }

    public Command indexReverse() {
        return runOnce(() -> {
            setpoint = IndexerConstants.kCollectorRPM.times(-1);
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM.times(-1));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            setpoint = RotationsPerSecond.of(0);
            indexerIO.stop();
        });
    }

    public void setRunning(boolean running) {
        if (running) {
            setpoint = IndexerConstants.kCollectorRPM;
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        } else {
            setpoint = RotationsPerSecond.of(0);
            indexerIO.stop();
        }
    }
}
