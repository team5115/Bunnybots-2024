package frc.team5115.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmFeedforward feedforward;
    private final PIDController pid;

    public Arm(ArmIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward = new ArmFeedforward(0.3, 0.35, 0.13509, 0.048686);
                pid = new PIDController(0.405, 0.0, 0.0);
                break;
            case SIM:
                feedforward = new ArmFeedforward(0.0, 0.35, 0.15, 0.0);
                pid = new PIDController(1.0, 0.0, 0.0);
                break;
            default:
                feedforward = new ArmFeedforward(0.0, 0.0, 0, 0.0);
                pid = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        pid.setTolerance(3.0);
        pid.setSetpoint(91.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/Setpoint Degrees", pid.getSetpoint());
        Logger.recordOutput("Arm/Actual Degrees", inputs.armAngle.getDegrees());
        Logger.recordOutput("Arm/At Setpoint?", pid.atSetpoint());

        // Update the pids and feedforward
        final double speed = pid.calculate(inputs.armAngle.getDegrees());
        double voltage = feedforward.calculate(inputs.armAngle.getRadians(), speed);
        voltage = MathUtil.clamp(voltage, -10, +10);

        io.setArmVoltage(voltage);
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> pid.atSetpoint()).withTimeout(timeout);
    }

    public Command setAngle(Rotation2d setpoint) {
        return Commands.runOnce(() -> pid.setSetpoint(setpoint.getDegrees()));
    }

    public Command goToAngle(Rotation2d setpoint, double timeout) {
        return setAngle(setpoint).andThen(waitForSetpoint(timeout));
    }

    public Command stow() {
        return setAngle(Rotation2d.fromDegrees(+66.0));
    }

    public Command prepareDispense() {
        return setAngle(Rotation2d.fromDegrees(+135.0));
    }

    public Command intake() {
        return setAngle(Rotation2d.fromDegrees(-24.0));
    }
}
