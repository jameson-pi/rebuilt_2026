package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.TunableTalonFX;

public class PIDRollerIOReal implements RollerIO {

    private final TunableTalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;
    private final Slot0Configs rollerPID;

    public PIDRollerIOReal() {
        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        rollerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rollerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        
        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        rollerMotor = new TunableTalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID, "rio", "Intake/RollerPID", rollerPID);
        rollerMotor.applyConfiguration(rollerMotorConfig);
    }

    public void setRollerSpeed(AngularVelocity speed) {
        rollerMotor.setControl(new VelocityVoltage(0).withVelocity(speed));
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();;
    }

    @Override
    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.kIntakeSpeed);
    }

    @Override
    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.kOuttakeSpeed);
    }

    @Override
    public void updateInputs(RollerIO.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = rollerMotor.get();
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValue();
        inputs.rollerVelocity = rollerMotor.getVelocity().getValue();
    }
}
