package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private IndexerIO indexerIO;
    private IndexerIOInputs indexerInputs;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
        this.indexerInputs = new IndexerIO.IndexerIOInputs();
    }

    public Command index() {
        return runOnce(() -> indexerIO.setVelocity(IndexerConstants.kCollectorRPM));
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> indexerIO.setCustomSpeed(speed));
    }

    public Command indexReverse() {
        return runOnce(() -> indexerIO.setVelocity(IndexerConstants.kCollectorRPM.times(-1)));
    }

    public Command stop() {
        return runOnce(() -> indexerIO.stop());
    }

    public void setRunning(boolean running) {
        if (running) {
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        } else {
            indexerIO.stop();
        }
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
    }
}
