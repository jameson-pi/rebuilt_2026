package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public Voltage motorOutput = Volts.of(0);
    }

    default void index() {}

    default void indexReverse() {}

    default void stop() {}

    default void setCustomSpeed(double speed) {}

    default void runAtSpeed() {}

    default void updateInputs(IndexerIOInputs indexerInputs) {}
}
