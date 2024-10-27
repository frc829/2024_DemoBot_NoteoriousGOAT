package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.RobotContainer.*;

public class StationaryRangedShot {
    public static final class Ranged {
        private static final class Constants {
            private static Measure<Distance> elevatorPosition = Meters.of(0.0);
            private static double topShooterPercent = -0.8;
            private static double bottomShooterPercent = 0.8;
            private static double transportPercent = 0.9;
            private static double singulatorPercent = -0.9;
            private static final double shooterTolerancePercent = 0.10;
            private static final double endOfShootDelay = 0.2;

            private static final double tolerance = 2;
            private static final PIDController rotationPIDController = new PIDController(0.10, 0, 0);
            private static final double[] distances = new double[] {
                    1.28,
                    2.00,
                    2.49,
                    3.00,
                    3.50,
                    4.01,
                    4.50
            };
            private static final double[] anglesDegrees = new double[] {
                    57.0,
                    43.0,
                    38.0,
                    32.5,
                    31.5,
                    28.0,
                    25.75
            };
            private static final Spline spline = MonotoneCubicSpline.createMonotoneCubicSpline(distances,
                    anglesDegrees);

            private static final Function<Measure<Distance>, Measure<Angle>> distanceToAngle = (distance) -> {
                return Degrees.of(spline.interpolate(distance.in(Meters)));
            };

            private static final Supplier<Double> angleAdjust = () -> {
                double angleDegrees = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
                if (angleDegrees >= 10) {
                    return 3.0;
                } else if (angleDegrees <= -10) {
                    return -1.5;
                } else {
                    return 0.0;
                }
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
                double angleAdjust = Constants.angleAdjust.get();
                poseAngle = poseAngle.plus(angle.get());
                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees() + angleAdjust);
                Constants.rotationPIDController.setTolerance(Constants.tolerance);
                return true;
            } else {
                SmartDashboard.putBoolean("Has Priority Tag", false);
                return false;
            }
        };

        private static final Supplier<Command> setDistanceAndAngleCommand = () -> {
            Runnable blank = () -> {
            };
            return Commands.run(blank).until(setDistanceAndAngle);
        };

        private static final Runnable rotateDriveToFoundTarget = () -> {
            var angle = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
            double omegaRadiansPerSecond = Constants.rotationPIDController.calculate(angle);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSecond);
            driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()).accept(chassisSpeeds);
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
                    .raceWith(Commands.waitSeconds(2));
            command.setName("Ranged Aim");
            return command;
        };

        private static final Supplier<Command> createAimCommandInitial = () -> {
            Command command = Commands.parallel(
                    rotateDriveToAlignTargetCommand.get(),
                    BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                            .apply(Constants.elevatorPosition),
                    BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(tiltAngle),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                    BasicCommands.BottomShooter.createSpinCommand
                            .apply(Constants.bottomShooterPercent))
                    .until(BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle));
            command.setName("Ranged Aim Initial");
            return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
            Command command = Commands.parallel(
                    Commands.run(driveSubsystem.stop, driveSubsystem),
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
                    Commands.runOnce(shooterTiltSubsystem.resetRelEncoderFromAbsolute, shooterTiltSubsystem),
                    TelemetryCommands.createSetRearCameraToFieldCommand.get(),
                    setDistanceAndAngleCommand.get(),
                    createAimCommandInitial.get(),
                    Commands.race(Commands.waitSeconds(0.25), Commands.run(driveSubsystem.stop, driveSubsystem)),
                    Commands.runOnce(shooterTiltSubsystem.resetRelEncoderFromAbsolute, shooterTiltSubsystem),
                    createAimCommand.get(),
                    createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
            return Commands.sequence(
                    Commands.runOnce(shooterTiltSubsystem.resetRelEncoderFromAbsolute, shooterTiltSubsystem),
                    TelemetryCommands.createSetRearCameraToFieldCommand.get(),
                    setDistanceAndAngleCommand.get(),
                    createAimCommandInitial.get(),
                    Commands.race(Commands.waitSeconds(0.4), Commands.run(driveSubsystem.stop, driveSubsystem)),
                    Commands.runOnce(shooterTiltSubsystem.resetRelEncoderFromAbsolute, shooterTiltSubsystem),
                    createAimCommand.get(),
                    createShootCommand.get()
                            .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)),
                    ShutOff.shutOffCommand.get());
        };
    }
}
