package frc.team5115.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.team5115.Constants;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim arm;
    private double voltage;

    public ArmIOSim() {
        final double randomStartRads = Math.random() * Math.PI - Math.toRadians(24);
        final double gearing = 5.0 * 5.0 * (24.0 / 42.0);
        arm =
                new SingleJointedArmSim(
                        DCMotor.getNEO(2),
                        gearing,
                        0.1,
                        0.7874,
                        Math.toRadians(-30),
                        Math.toRadians(+150),
                        true,
                        randomStartRads);
        voltage = 0.0;
    }

    public void updateInputs(ArmIOInputs inputs) {
        arm.update(Constants.LOOP_PERIOD_SECS);

        inputs.armAngle = Rotation2d.fromRadians(arm.getAngleRads());
        inputs.appliedVolts = voltage;
        inputs.currentAmps = arm.getCurrentDrawAmps();
        inputs.armVelocityRPM = arm.getVelocityRadPerSec() * 60.0;
    }

    public void setArmVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        arm.setInputVoltage(voltage);
    }
}
