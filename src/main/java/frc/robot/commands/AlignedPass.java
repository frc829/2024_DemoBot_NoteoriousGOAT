package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.RobotContainer.*;

public abstract class AlignedPass {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(45);
                private static Measure<Distance> elevatorPosition = Meters.of(0.0);
                private static double topShooterPercent = -0.5;
                private static double bottomShooterPercent = 0.5;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.9;
                private static final double shooterTolerancePercent = 0.10;
                private static final double endOfShootDelay = 0.2;

                private static final double tolerance = 2;
                private static final PIDController rotationPIDController = new PIDController(0.10, 0, 0);

        }

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
                return BasicCommands.Elevator.createAtPositionCondition.apply(Constants.elevatorPosition)
                                .getAsBoolean() &&
                                BasicCommands.Tilt.createAtAngleCondition.apply(Constants.tiltAngle)
                                                .getAsBoolean();

        };

        private static final Supplier<Command> createAimCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent))
                                .until(elevatorAndTiltAtPositionCondition);
                command.setName("Fender Aim");
                return command;
        };

        private static final Supplier<Command> createSpinUp = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent))
                                .until(shootersAtSpeed);
                command.setName("Fender Spin Up");
                return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent),
                                BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                                BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));
                command.setName("Fender Shoot");
                return command;
        };

        private static final Supplier<Command> setPriorityTargetCommand = () -> {
                Runnable setPriorityTarget = () -> {
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                telemetrySubsystem.setPriorityTargetsFromFieldDetectors.get(1).accept(13);
                        } else {
                                telemetrySubsystem.setPriorityTargetsFromFieldDetectors.get(1).accept(14);
                        }
                };
                Command command = Commands.runOnce(setPriorityTarget, telemetrySubsystem);
                return command;
        };

        public static final Supplier<Command> resetPriorityTargetCommand = () -> {
                Runnable setPriorityTarget = () -> {
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                telemetrySubsystem.setPriorityTargetsFromFieldDetectors.get(1).accept(4);
                        } else {
                                telemetrySubsystem.setPriorityTargetsFromFieldDetectors.get(1).accept(7);
                        }
                };
                Command command = Commands.runOnce(setPriorityTarget, telemetrySubsystem);
                return command;
        };

        private static final Runnable rotateDriveToFoundTarget = () -> {
                var angle = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
                double omegaRadiansPerSecond = Constants.rotationPIDController.calculate(angle);
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSecond);
                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d())
                                .accept(chassisSpeeds);
        };

        private static final Supplier<Command> rotateDriveToAlignTargetCommand = () -> {
                Command command = Commands.run(rotateDriveToFoundTarget, driveSubsystem);
                command.setName("Rotate To Align Target");
                return command;
        };


        private static final BooleanSupplier setDistanceAndAngle = () -> {
                var distance = telemetrySubsystem.priorityTargetDistance.get();
                var angle = telemetrySubsystem.priorityTargetRotation.get();
                if (distance.isPresent() && angle.isPresent()) {
                        SmartDashboard.putBoolean("Has Priority Tag", true);
                        Rotation2d poseAngle = telemetrySubsystem.poseEstimate.get().getRotation();
                        poseAngle = poseAngle.plus(angle.get());
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees() - 5.0);
                        } else {
                                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees() + 5.0);
                        }
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

        private static final Supplier<Command> createAimCommandInitial = () -> {
                Command command = Commands.parallel(
                                rotateDriveToAlignTargetCommand.get(),
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent))
                                .until(() -> Constants.rotationPIDController.atSetpoint());
                command.setName("Ranged Aim Initial");
                return command;
        };

        public static final Supplier<Command> create = () -> {
                return Commands.sequence(
                                TelemetryCommands.createSetRearCameraToFieldCommand.get(),
                                setPriorityTargetCommand.get(),
                                setDistanceAndAngleCommand.get(),
                                createAimCommandInitial.get(),
                                Commands.race(Commands.waitSeconds(0.75),
                                                Commands.run(driveSubsystem.stop, driveSubsystem)),
                                createAimCommand.get(),
                                createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
                return Commands.sequence(
                                createAimCommand.get(),
                                createSpinUp.get(),
                                createShootCommand.get()
                                                .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)));
        };
}