package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.utility.Spline;
import com.utility.Spline.MonotoneCubicSpline;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.RobotContainer.*;

public class MovingRangedShot {
    public static final class Ranged {
        private static final class Constants {
            private static Measure<Distance> elevatorPosition = Meters.of(0.0);
            private static double topShooterPercent = -0.7;
            private static double bottomShooterPercent = 0.7;
            private static double transportPercent = 0.9;
            private static double singulatorPercent = -0.9;
            private static final double shooterTolerancePercent = 0.10;
            private static final double endOfShootDelay = 0.2;

            private static final double tolerance = 2;
            private static final PIDController rotationPIDController = new PIDController(0.05, 0, 0);
            private static final double[] distances = new double[] {
                    1.28,
                    2.00,
                    3.00,
                    4.00,
                    5.00
            };
            private static final double[] anglesDegrees = new double[] {
                    57.0,
                    44.0,
                    36.0,
                    30.0,
                    27.0
            };
            private static final Spline spline = MonotoneCubicSpline.createMonotoneCubicSpline(distances,
                    anglesDegrees);

            private static final Function<Measure<Distance>, Measure<Angle>> distanceToAngle = (distance) -> {
                return Degrees.of(spline.interpolate(distance.in(Meters)));
            };
        }

        private static final MutableMeasure<Distance> targetDistance = MutableMeasure.zero(Meters);
        private static final MutableMeasure<Angle> tiltAngle = MutableMeasure.zero(Degrees);

        private static final BooleanSupplier shootersAtSpeed = () -> {
            boolean topCondition = MathUtil.isNear(
                    Math.abs(Constants.topShooterPercent),
                    Math.abs(topShooterSubsystem.velocity.in(Value)),
                    Constants.shooterTolerancePercent);
            boolean bottomCondition = MathUtil.isNear(
                    Math.abs(Constants.bottomShooterPercent),
                    Math.abs(bottomShooterSubsystem.velocity.in(Value)),
                    Constants.shooterTolerancePercent);
            boolean condition = topCondition && bottomCondition;
            SmartDashboard.putBoolean("ShooterAt Speed", condition);
            return condition;
        };

        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
            boolean condition = BasicCommands.Elevator.createAtPositionCondition.apply(Constants.elevatorPosition)
                    .getAsBoolean() &&
                    BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                            .getAsBoolean();
            SmartDashboard.putBoolean("Elevator and Tilt At Position", condition);
            return condition;
        };

        private static final BooleanSupplier alignedWithTarget = () -> {
            boolean condition = Constants.rotationPIDController.atSetpoint();
            SmartDashboard.putBoolean("Aligned with Target", condition);
            return condition;
        };

        private static final BooleanSupplier elevatorTiltShooterDriveAtPositionCondition = () -> {
            return elevatorAndTiltAtPositionCondition.getAsBoolean() && alignedWithTarget.getAsBoolean()
                    && shootersAtSpeed.getAsBoolean();
        };

        private static final BooleanSupplier setDistanceAndAngle = () -> {
            var distance = telemetrySubsystem.priorityTargetDistance.get();
            var angle = telemetrySubsystem.priorityTargetRotation.get();
            if (distance.isPresent() && angle.isPresent()) {
                SmartDashboard.putBoolean("Has Priority Tag", true);
                targetDistance.mut_setMagnitude(distance.get());
                var aimAngle = Constants.distanceToAngle.apply(targetDistance);
                tiltAngle.mut_setMagnitude(aimAngle.in(Degrees));
                Rotation2d poseAngle = telemetrySubsystem.poseEstimate.get().getRotation();
                poseAngle = poseAngle.plus(angle.get());
                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees());
                Constants.rotationPIDController.setTolerance(Constants.tolerance);
                return true;
            } else {
                SmartDashboard.putBoolean("Has Priority Tag", false);
                return false;
            }
        };

        private static final Supplier<Command> setDistanceAndAngleCommand = () -> {
            Command driveSlow = DriveCommands.createFieldCentricDriveOriginSlowCommand.get();
            return driveSlow.until(setDistanceAndAngle);
        };

        private static final ChassisSpeeds speeds = new ChassisSpeeds();
        private static final Runnable rotateDriveToFoundTarget = () -> {
            var angle = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
            double omegaRadiansPerSecond = Constants.rotationPIDController.calculate(angle);
            double flipper = 0.35;
            var color = DriverStation.getAlliance();
            if (color.isPresent() && color.get() == Alliance.Red) {
                flipper *= -1;
            }
            speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                    .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
            speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                    .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
            speeds.vxMetersPerSecond *= flipper;
            speeds.vyMetersPerSecond *= flipper;
            ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                    RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
            speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
            speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
            speeds.omegaRadiansPerSecond = omegaRadiansPerSecond;
            driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()).accept(speeds);
        };

        private static final Supplier<Command> rotateDriveToAlignTargetCommand = () -> {
            Command command = Commands.run(rotateDriveToFoundTarget, driveSubsystem);
            command.setName("Rotate To Align Target");
            return command;
        };

        private static final Supplier<Command> createAimCommand = () -> {
            Command command = Commands.parallel(
                    rotateDriveToAlignTargetCommand.get(),
                    BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                            .apply(Constants.elevatorPosition),
                    BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(tiltAngle),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                    BasicCommands.BottomShooter.createSpinCommand
                            .apply(Constants.bottomShooterPercent))
                    .until(elevatorTiltShooterDriveAtPositionCondition)
                    .raceWith(Commands.waitSeconds(0.5));
            command.setName("Ranged Aim");
            return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
            Command command = Commands.parallel(
                    rotateDriveToAlignTargetCommand.get(),
                    BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                            .apply(Constants.elevatorPosition),
                    BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(tiltAngle),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                    BasicCommands.BottomShooter.createSpinCommand
                            .apply(Constants.bottomShooterPercent),
                    BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                    BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));
            command.setName("Ranged Shoot");
            return command;
        };

        public static final Supplier<Command> create = () -> {
            return Commands.sequence(
                    TelemetryCommands.createSetRearCameraToFieldCommandDriveSlow.get(),
                    setDistanceAndAngleCommand.get(),
                    createAimCommand.get(),
                    createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
            return Commands.sequence(
                    TelemetryCommands.createSetRearCameraToFieldCommand.get(),
                    setDistanceAndAngleCommand.get(),
                    createAimCommand.get(),
                    createShootCommand.get()
                            .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)));
        };
    }
}
