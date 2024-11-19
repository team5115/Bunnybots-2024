package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.drive.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    public static Command prepareDispense(Dispenser dispenser, Arm arm) {
        return Commands.sequence(dispenser.stop(), arm.prepareDispense());
    }

    public static Command score(Dispenser dispenser, Arm arm, Drivetrain drivetrain) {
        return Commands.sequence(drivetrain.alignPoseA(), quickScore(dispenser, arm, drivetrain));
    }

    public static Command quickScore(Dispenser dispenser, Arm arm, Drivetrain drivetrain) {
        return Commands.sequence(
                        arm.prepareDispense(),
                        drivetrain.alignPoseB(),
                        dispenser.dispense(),
                        dispenser.waitTillNoCanister(),
                        dispenser.stop(),
                        drivetrain.alignPoseA(),
                        arm.stow())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command dispense(Dispenser dispenser, Arm arm) {
        return Commands.sequence(
                        dispenser.dispense(),
                        dispenser.waitTillNoCanister().alongWith(Commands.waitSeconds(1.0)),
                        dispenser.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command intakeUntilCanister(Dispenser dispenser, Arm arm) {
        return Commands.sequence(
                dispenser.intake(), arm.intake(), dispenser.waitTillCanister(), dispenser.stop());
    }

    public static Command stow(Dispenser dispenser, Arm arm) {
        return Commands.parallel(dispenser.stop(), arm.stow());
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    drivetrain.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED,
                                    linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED,
                                    omega * SwerveConstants.MAX_ANGULAR_SPEED,
                                    drivetrain.isRedAlliance()
                                            ? drivetrain.getRotation().plus(new Rotation2d(Math.PI))
                                            : drivetrain.getRotation()));
                },
                drivetrain);
    }
}
