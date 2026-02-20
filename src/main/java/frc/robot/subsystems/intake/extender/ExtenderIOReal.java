package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final CurrentLimitsConfigs currentConfig;
    private final TalonFXConfiguration extenderMotorConfig;
    private final Slot0Configs extenderPID;
    private Angle setpoint;

    public ExtenderIOReal() {
        this.setpoint = Rotations.of(0.0);

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
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        extenderMotorConfig.Feedback.SensorToMechanismRatio = ExtenderConstants.kGearing;

        extenderMotor.applyConfiguration(extenderMotorConfig);
        extenderMotor.getConfigurator().apply(currentConfig);
    }

    public void setPosition(Angle position) {
        this.setpoint = Rotations.of(Math.max(
                ExtenderConstants.kExtenderMinAngle.in(Rotations),
                Math.min(ExtenderConstants.kExtenderMaxAngle.in(Rotations), position.in(Rotations))));
        extenderMotor.setControl(new PositionVoltage(this.setpoint));
    }

    public Angle getPosition() {
        return extenderMotor.getPosition().getValue();
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Rotations))
                < ExtenderConstants.kExtenderTolerance.in(Rotations);
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
        if (isAtAngle(ExtenderConstants.kExtenderStowAngle)) {
            extend();
        } else if (isAtAngle(ExtenderConstants.kExtenderIntakeAngle)) {
            retract();
        }
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended = isExtended().getAsBoolean();
        inputs.isRetracted = isRetracted().getAsBoolean();
        inputs.position = getPosition();
        inputs.setpoint = setpoint;
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.atTarget = atTarget().getAsBoolean();
    }

    @Override
    public void periodic() {
        extenderMotor.updateTunableGains();
    }
}
