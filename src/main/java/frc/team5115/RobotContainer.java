package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.arm.ArmIO;
import frc.team5115.subsystems.arm.ArmIOSim;
import frc.team5115.subsystems.arm.ArmIOSparkMax;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.dispenser.DispenserIO;
import frc.team5115.subsystems.dispenser.DispenserIOSim;
import frc.team5115.subsystems.dispenser.DispenserIOSparkMax;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final GyroIO gyro;
    private final Drivetrain drivetrain;
    private final PhotonVision vision;
    private final Arm arm;
    private final Dispenser dispenser;

    // Controller
    private final CommandXboxController joyDrive = new CommandXboxController(0);
    private final CommandXboxController joyManip = new CommandXboxController(1);
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Shuffleboard
    private final GenericEntry canisterDetectedEntry;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                gyro = new GyroIONavx();
                arm = new Arm(new ArmIOSparkMax());
                dispenser = new Dispenser(new DispenserIOSparkMax());

                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                vision = new PhotonVision(drivetrain);
                // vision = null;
                canisterDetectedEntry =
                        Shuffleboard.getTab("SmartDashboard").add("Has Canister?", false).getEntry();
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                arm = new Arm(new ArmIOSim());
                dispenser = new Dispenser(new DispenserIOSim());

                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                vision = null;
                canisterDetectedEntry = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                arm = new Arm(new ArmIO() {});
                dispenser = new Dispenser(new DispenserIO() {});

                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = null;
                canisterDetectedEntry = null;
                break;
        }

        // Register auto commands for pathplanner
        // PhotonVision is passed in here to prevent warnings, i.e. "unused variable: vision"
        registerCommands(drivetrain, vision, arm, dispenser);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Forward)",
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Reverse)",
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Arm SysId (Quasistatic Forward)", arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Shooter SysId (Quasistatic Reverse)",
                arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Arm SysId (Dynamic Forward)", arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Arm SysId (Dynamic Reverse)", arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.start().onTrue(resetFieldOrientation());

        joyManip.a().onTrue(DriveCommands.intakeUntilCanister(dispenser, arm));

        joyManip.b().onTrue(DriveCommands.score(dispenser, arm, drivetrain));

        joyManip.y().onTrue(DriveCommands.stow(dispenser, arm));

        joyManip.x().onTrue(DriveCommands.prepareDispense(dispenser, arm));

        joyManip.rightBumper().onTrue(DriveCommands.stackSequence(dispenser, arm));

        joyManip.rightTrigger().onTrue(dispenser.dispense()).onFalse(dispenser.stop());
        joyManip.leftTrigger().onTrue(dispenser.intake()).onFalse(dispenser.stop());
    }

    public void teleopPeriodic() {
        // arm.goAtVoltage(-joyManip.getLeftY());
    }

    public void robotPeriodic() {
        if (canisterDetectedEntry != null) {
            canisterDetectedEntry.setBoolean(dispenser.isDetectingCanister());
        }
    }

    /**
     * Register commands for pathplanner autos
     *
     * @param drivetrain
     * @param vision
     * @param arm
     * @param dispenser
     */
    public static void registerCommands(
            Drivetrain drivetrain, PhotonVision vision, Arm arm, Dispenser dispenser) {

        NamedCommands.registerCommand("Dispense", DriveCommands.quickScore(dispenser, arm, drivetrain));

        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilCanister(dispenser, arm));

        NamedCommands.registerCommand("Stack", DriveCommands.stackSequence(dispenser, arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private Command resetFieldOrientation() {
        return Commands.runOnce(
                        () -> {
                            drivetrain.setPose(
                                    new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d()));
                            drivetrain.offsetGyro();
                        },
                        drivetrain)
                .ignoringDisable(true);
    }

    private Command resetRobotPose() {
        return Commands.runOnce(
                        () -> {
                            drivetrain.setPose(
                                    new Pose2d(
                                            new Translation2d(
                                                    drivetrain.isRedAlliance() ? Constants.FIELD_WIDTH_METERS - 1.35 : 1.35,
                                                    5.55),
                                            new Rotation2d()));
                        },
                        drivetrain)
                .ignoringDisable(true);
    }
}
