package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ExtenderIOSim implements ExtenderIO {

    private final TalonFXConfiguration extenderMotorConfig;
    private final TunableTalonFX extenderMotor;
    private final Slot0Configs extenderPID;
    private final TalonFXSimState extenderMotorSim;
    private final LoggedMechanism2d armMech;
    private final LoggedMechanismRoot2d armMechRoot;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d setpointArmLigament;
    private final CurrentLimitsConfigs currentConfig;
    private final SingleJointedArmSim armSim;
    private Angle setpoint;

    public ExtenderIOSim() {
        setpoint = Degrees.of(0.0);

        extenderPID = new Slot0Configs();
        extenderPID.kP = ExtenderConstants.PIDF.kP;
        extenderPID.kI = ExtenderConstants.PIDF.kI;
        extenderPID.kD = ExtenderConstants.PIDF.kD;

        extenderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/ExtenderPID", extenderPID);

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = ExtenderConstants.MotorConfig.kStatorCurrentLimit.in(Amps);

        extenderMotorConfig = new TalonFXConfiguration();
        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ExtenderConstants.MotorConfig.kRampPeriod;
        extenderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = ExtenderConstants.MotorConfig.kPeakForwardTorque;
        extenderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = ExtenderConstants.MotorConfig.kPeakReverseTorque;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderMotorConfig.Feedback.SensorToMechanismRatio = ExtenderConstants.kGearing;

        extenderMotor.applyConfiguration(extenderMotorConfig);
        extenderMotor.getConfigurator().apply(currentConfig);

        extenderMotorSim = extenderMotor.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ExtenderConstants.kGearing,
                ExtenderConstants.kMOI.in(KilogramMetersSquaredPerSecond),
                ExtenderConstants.kExtenderArmLength.in(Meters),
                ExtenderConstants.kExtenderStowAngle.in(Radians),
                ExtenderConstants.kExtenderIntakeAngle.in(Radians),
                false,
                ExtenderConstants.kExtenderIntakeAngle.in(Radians),
                0.0,
                0.0);

        armMech = new LoggedMechanism2d(5, 5);
        armMechRoot = armMech.getRoot("IntakeSimulation", 3, 3);
        armLigament = new LoggedMechanismLigament2d("arm", 2, ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        setpointArmLigament = new LoggedMechanismLigament2d("setpoint", 2, setpoint.in(Degrees));
        armMechRoot.append(armLigament);
        armMechRoot.append(setpointArmLigament);
    }

    public void setPosition(Angle position) {
        this.setpoint = Degrees.of(Math.max(
                ExtenderConstants.kExtenderMinAngle.in(Degrees),
                Math.min(ExtenderConstants.kExtenderMaxAngle.in(Degrees), position.in(Degrees))));
        extenderMotor.setControl(new PositionVoltage(this.setpoint));
    }

    public Angle getPosition() {
        return extenderMotor.getPosition().getValue();
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Degrees)) < ExtenderConstants.kExtenderTolerance.in(Degrees);
    }

    @Override
    public void zero() {
        extenderMotor.setPosition(0.0);
    }

    @Override
    public void extend() {
        setPosition(ExtenderConstants.kExtenderIntakeAngle);
    }

    @Override
    public void retract() {
        setPosition(ExtenderConstants.kExtenderStowAngle);
    }

    @Override
    public BooleanSupplier isExtended() {
        return () -> isAtAngle(ExtenderConstants.kExtenderIntakeAngle);
    }

    @Override
    public BooleanSupplier isRetracted() {
        return () -> isAtAngle(ExtenderConstants.kExtenderStowAngle);
    }

    @Override
    public BooleanSupplier atTarget() {
        return () -> isAtAngle(setpoint);
    }

    @Override
    public void goToSiftAngleOne() {
        setPosition(ExtenderConstants.kExtenderSiftAngleOne);
    }

    @Override
    public void goToSiftAngleTwo() {
        setPosition(ExtenderConstants.kExtenderSiftAngleTwo);
    }

    @Override
    public void toggle() {
        if (isAtAngle(ExtenderConstants.kExtenderIntakeAngle)) {
            retract();
        } else if (isAtAngle(ExtenderConstants.kExtenderStowAngle)) {
            extend();
        }
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended = isExtended().getAsBoolean();
        inputs.isRetracted = isRetracted().getAsBoolean();
        inputs.position = getPosition();
        inputs.setpoint = setpoint;
        inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        inputs.motorVoltage = Volts.of(extenderMotorSim.getMotorVoltage());
        inputs.motorCurrent = Amps.of(armSim.getCurrentDrawAmps());
        inputs.motorTemp = Celsius.of(25.0);
        inputs.atTarget = atTarget().getAsBoolean();
    }

    @Override
    public void periodic() {
        armSim.setInputVoltage(extenderMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);
        extenderMotor.setPosition(Radians.of(armSim.getAngleRads()));

        armLigament.setAngle(getPosition());
        setpointArmLigament.setAngle(setpoint);
        Logger.recordOutput("Intake/2D-Simulation", armMech);

        extenderMotor.updateTunableGains();
    }
}
