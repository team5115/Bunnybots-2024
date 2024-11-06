package frc.team5115.subsystems.dispenser;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;


public class Dispenser extends SubsystemBase {
    private final DispenserIO io;
    private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;

    private double setpointRPM;

    public Dispenser(DispenserIO io) {
        this.io = io;
    

    switch (Constants.currentMode) {
        case REAL:
        case REPLAY:
            feedforward = new SimpleMotorFeedforward(0, 0, 0); //TODO: Fill in values 
            pid = new PIDController(0.405, 0.0, 0.0);
            break;
        case SIM:
            feedforward = new SimpleMotorFeedforward(0, 0, 0); //TODO: Fill in values 
            pid = new PIDController(0.5, 0.0, 0.0);
            break;
        default:
            feedforward = new SimpleMotorFeedforward(0, 0, 0); //TODO: Fill in values
            pid = new PIDController(0.0, 0.0, 0.0);
            break;
    }

    pid.setTolerance(5);
    pid.setSetpoint(75.0);
  
    }
    public Command stop() {
        return Commands.runOnce(() -> setpointRPM = 0, this);
    }

     private void setSetpoint(double rpm) {
        setpointRPM = rpm;
        pid.setSetpoint(rpm);
    }
    
    public Command spinToSpeed() {
            return Commands.runOnce(() -> setSetpoint(5000), this)
                    .andThen(Commands.waitUntil(() -> pid.atSetpoint()));
        }
} 


