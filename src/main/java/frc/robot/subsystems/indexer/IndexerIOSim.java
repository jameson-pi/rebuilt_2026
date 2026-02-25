package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final double ROLLER_MASS_KG = 0.25;
    private static final double ROLLER_RADIUS_M = 0.02;
    private static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;
    private static final double ROLLER_GEARING = 1.0;

    private final FlywheelSim sim;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private double setpointRPM = 0.0;
    private double appliedVolts = 0.0;

    public IndexerIOSim() {
        var plant = LinearSystemId.createFlywheelSystem(MOTOR, ROLLER_MOI, ROLLER_GEARING);
        sim = new FlywheelSim(plant, MOTOR);

        controller = new PIDController(
                IndexerConstants.SimPID.kP, IndexerConstants.SimPID.kI, IndexerConstants.SimPID.kD);

        double freeSpeedRPM = MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double kv = 12.0 / freeSpeedRPM;
        feedforward = new SimpleMotorFeedforward(0.0, kv, 0.0);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        setpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        setpointRPM = 0.0;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }

    private double currentLimitedVoltage(double desiredVolts) {
        double velocityRadPerSec = sim.getAngularVelocityRPM() * 2.0 * Math.PI / 60.0;
        double backEmf = velocityRadPerSec * (12.0 / MOTOR.freeSpeedRadPerSec);
        double motorResistance = 12.0 / MOTOR.stallCurrentAmps;
        double vMax = Math.min(
                12.0,
                backEmf + IndexerConstants.MotorConfigurationConfigs.PeakForwardTorqueCurrent * motorResistance);
        double vMin = Math.max(
                -12.0,
                backEmf - IndexerConstants.MotorConfigurationConfigs.PeakReverseTorqueCurrent * motorResistance);
        return MathUtil.clamp(desiredVolts, vMin, vMax);
    }

    @Override
    public void setCustomSpeed(double speed) {
        appliedVolts = currentLimitedVoltage(speed * 12.0);
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        double ff = feedforward.calculate(setpointRPM);
        double fb = controller.calculate(sim.getAngularVelocityRPM(), setpointRPM);
        appliedVolts = currentLimitedVoltage(ff + fb);

        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        indexerInputs.motorOutput = Volts.of(appliedVolts);
        indexerInputs.motorVelocity = RPM.of(sim.getAngularVelocityRPM());
    }
}
