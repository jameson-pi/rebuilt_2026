package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

    public static final double kCollectorSpeed = 0.5; // Adjust speed as needed
    public static final AngularVelocity kCollectorRPM = RotationsPerSecond.of(300); // Adjust RPM as needed

    public static final String canBus = "rio";

    public final class MotorConfigurationConfigs {
        public static final double VoltageClosedLoopRampPeriod = 0.02;
        public static final double PeakForwardTorqueCurrent = 40;
        public static final double PeakReverseTorqueCurrent = 40;
        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorNeutralMode = NeutralModeValue.Brake;
    }

    public final class PID {
        public static final double kP = 0.1; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }

    public final class SimPID {
        public static final double kP = 0.1; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }
}
