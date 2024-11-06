package frc.team5115.subsystems.dispenser;
import org.littletonrobotics.junction.AutoLog;

public interface DispenserIO {
    @AutoLog
    public static class DispenserIOInputs{
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
     public default void updateInputs(DispenserIOInputs inputs) {}
     public default void setDispenserVoltage(double volts) {}
}