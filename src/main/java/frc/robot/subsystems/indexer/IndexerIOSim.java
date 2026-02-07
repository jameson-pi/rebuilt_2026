package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOSim implements IndexerIO {
    private final TalonFX indexerMotor;
    private final TalonFXSimState indexerMotorSim;
    private final TalonFXConfiguration indexerMotorConfig;
    private final LoggedNetworkNumber indexerMotorOutput;
    private final LoggedNetworkNumber indexerMotorReverseOutput;

    public IndexerIOSim() {
        indexerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kIndexerMotorID);
        indexerMotorConfig = new TalonFXConfiguration();

        indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        indexerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        indexerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;
        indexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotor.getConfigurator().apply(indexerMotorConfig);
        indexerMotorSim = indexerMotor.getSimState();

        indexerMotorOutput = new LoggedNetworkNumber("Indexer/IndexerMotorOutput", IndexerConstants.kCollectorSpeed);
        indexerMotorReverseOutput = new LoggedNetworkNumber("Indexer/IndexerMotorReverseOutput", 0);
    }

    @Override
    public void index() {
        double value = indexerMotorOutput.get();
        indexerMotor.set(Math.max(0, Math.min(Math.abs(value), 1.0)));
    }

    @Override
    public void indexReverse() {
        double value = indexerMotorReverseOutput.get();
        indexerMotor.set(Math.max(0, Math.min(Math.abs(value), 1.0)) * -1);
    }

    @Override
    public void stop() {
        indexerMotor.set(0);
    }

    public void setCustomSpeed(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        indexerInputs.motorOutput = Volts.of(indexerMotorSim.getMotorVoltage());
    }
}
