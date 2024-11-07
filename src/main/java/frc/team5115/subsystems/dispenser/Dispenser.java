package frc.team5115.subsystems.dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Dispenser extends SubsystemBase {
    private final DispenserIO io;
    private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

    public Dispenser(DispenserIO dispenserIO) {
        this.io = dispenserIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Dispenser", inputs);
    }

    public Command stop() {
        return spin(0);
    }

    public Command spin(double percent) {
        return Commands.runOnce(() -> io.setDispenserPercent(percent), this);
    }

    public Command dispense() {
        return spin(+1);
    }

    public Command intake() {
        return spin(-1);
    }

    public Command waitTillCanister(){
        return Commands.waitUntil(() -> inputs.detecting);
    }

    public Command waitTillNoCanister(){
        return Commands.waitUntil(() -> !inputs.detecting);
    }
}
