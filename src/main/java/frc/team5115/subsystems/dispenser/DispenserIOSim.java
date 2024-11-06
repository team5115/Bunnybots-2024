package frc.team5115.subsystems.dispenser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DispenserIOSim implements DispenserIO{
    public DCMotorSim dispenser;
    private double dispenserAppliedVolts;

    public DispenserIOSim(){
        dispenser = new DCMotorSim(DCMotor.getNEO(1), 0, 0);
    }

    @Override
    public void setDispenserVoltage(double speed){
        dispenserAppliedVolts = MathUtil.clamp(speed * 12, -12.0, +12.0);
        dispenser.setInputVoltage(dispenserAppliedVolts);
    }

    @Override
    public void updateInputs(DispenserIOInputs inputs) {
        // TODO Auto-generated method stub
        inputs.velocityRPM = dispenser.getAngularVelocityRPM();
        inputs.currentAmps = dispenser.getCurrentDrawAmps();
        inputs.appliedVolts = dispenserAppliedVolts;
    }
}
