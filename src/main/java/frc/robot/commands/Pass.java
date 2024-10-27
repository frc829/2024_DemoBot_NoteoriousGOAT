package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;
import static frc.robot.RobotContainer.bottomShooterSubsystem;
import static frc.robot.RobotContainer.topShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Pass {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(50);
                private static Measure<Distance> elevatorPosition = Meters.of(0.0);
                private static double topShooterPercent = -0.5;
                private static double bottomShooterPercent = 0.5;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.9;
                private static final double shooterTolerancePercent = 0.10;
                private static final double endOfShootDelay = 0.2;

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

        public static final Supplier<Command> create = () -> {
                return Commands.sequence(createAimCommand.get(), createSpinUp.get(), createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
                return Commands.sequence(
                                createAimCommand.get(),
                                createSpinUp.get(),
                                createShootCommand.get()
                                                .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)));
        };
}