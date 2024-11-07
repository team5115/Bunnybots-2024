package frc.team5115.subsystems.dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Dispenser extends SubsystemBase {
    private final DispenserIO io;
    private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

    private double setpointRPM;

    public Dispenser(DispenserIO dispenserIO) {
        this.io = dispenserIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Dispenser", inputs);
        Logger.recordOutput("Dispenser/Setpoint RPM", setpointRPM);
    }

    public Command stop() {
        return Commands.runOnce(() -> setpointRPM = 0, this);
    }

    private void setPercent(double percent) {
        io.setDispenserPercent(percent);
    }

    public Command spin(double percent) {
        return Commands.runOnce(() -> setPercent(percent), this);
    }

    public Command dispense() {
        return Commands.runOnce(() -> setPercent(+1), this);
    }

    public Command intake() {
        return Commands.runOnce(() -> setPercent(-1), this);
    }
}
