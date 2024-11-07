package frc.team5115.subsystems.dispenser;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.team5115.Constants;

public class DispenserIOSparkMax implements DispenserIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public DispenserIOSparkMax() {
        motor = new CANSparkMax(Constants.DISPENSER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        motor.restoreFactoryDefaults();
        motor.setClosedLoopRampRate(0.1);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
    }

    @Override
    public void updateInputs(DispenserIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setDispenserVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setDispenserPercent(double percent) {
        motor.set(percent);
    }
}
