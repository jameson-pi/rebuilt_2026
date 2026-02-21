package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOSim implements IndexerIO {
    private final TalonFX indexerMotor;
    private final TalonFXSimState indexerMotorSim;
    private final PIDController indexerPIDController =
            new PIDController(IndexerConstants.SimPID.kP, IndexerConstants.SimPID.kI, IndexerConstants.SimPID.kD);
    private final TalonFXConfiguration indexerMotorConfig;
    private final LoggedNetworkNumber indexerMotorOutput;
    private final LoggedNetworkNumber indexerMotorReverseOutput;

    public IndexerIOSim() {
        indexerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kIndexerMotorID);
        indexerMotorConfig = new TalonFXConfiguration();

        indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
                IndexerConstants.MotorConfigurationConfigs.VoltageClosedLoopRampPeriod;
        indexerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakForwardTorqueCurrent;
        indexerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakReverseTorqueCurrent;
        indexerMotorConfig.MotorOutput.Inverted = IndexerConstants.MotorConfigurationConfigs.MotorInverted;
        indexerMotorConfig.MotorOutput.NeutralMode = IndexerConstants.MotorConfigurationConfigs.MotorNeutralMode;
        indexerMotor.getConfigurator().apply(indexerMotorConfig);
        indexerMotorSim = indexerMotor.getSimState();

        indexerMotorOutput = new LoggedNetworkNumber("Indexer/IndexerMotorOutput", IndexerConstants.kCollectorSpeed);
        indexerMotorReverseOutput = new LoggedNetworkNumber("Indexer/IndexerMotorReverseOutput", 0);
    }

    @Override
    public void stop() {
        indexerMotor.set(0);
    }

    @Override
    public void setCustomSpeed(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        indexerInputs.motorOutput = Volts.of(indexerMotorSim.getMotorVoltage());
    }
}
