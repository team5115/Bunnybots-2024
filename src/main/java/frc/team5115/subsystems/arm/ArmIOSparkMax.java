package frc.team5115.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        motor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360);
        absoluteEncoder.setInverted(true);
        motor.setInverted(false);
        motor.setSmartCurrentLimit(50);
        motor.burnFlash();
    }

    @Override
    public void setArmVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelocityRPM = absoluteEncoder.getVelocity() * 60.0;
        final var x = absoluteEncoder.getPosition() - 24.4;
        inputs.armAngle = Rotation2d.fromDegrees(x > 300 ? x - 360 : x);
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    }
}
