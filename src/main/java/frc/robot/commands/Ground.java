package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.innerIntakeSubsystem;
import static frc.robot.RobotContainer.notedLoadedSubsystem;
import static frc.robot.RobotContainer.outerIntakeSubsystem;
import static frc.robot.RobotContainer.transportSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public abstract class Ground {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(55);
                private static Measure<Distance> elevatorPosition = Meters.of(0);
                private static double transportPercent = 0.9;
                private static double innerIntakePercent = 0.9;
                private static double outerIntakePercent = 0.9;
                private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                        return BasicCommands.Elevator.createAtPositionCondition.apply(elevatorPosition)
                                        .getAsBoolean() &&
                                        BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                                                        .getAsBoolean();

                };
        }

        private static final Supplier<Command> elevatorAndTiltAtPositionCommand = () -> {
                return Commands.parallel(
                                BasicCommands.Elevator.createSetElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.tiltAngle))
                                .until(Constants.elevatorAndTiltAtPositionCondition);
        };

        private static final Supplier<Command> intakeCommand = () -> {
                return Commands.parallel(
                                BasicCommands.Elevator.createHoldElevatorCommand.get(),
                                BasicCommands.Tilt.createHoldTiltCommand.get(),
                                BasicCommands.OuterIntake.createSpinCommand.apply(Constants.outerIntakePercent),
                                BasicCommands.InnerIntake.createSpinCommand.apply(Constants.innerIntakePercent),
                                BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent),
                                BasicCommands.Singulator.createSpinCommand.apply(0.0));
        };

        private static final Supplier<Command> stop = () -> {
                return Commands.parallel(
                                Commands.runOnce(outerIntakeSubsystem.stop, outerIntakeSubsystem),
                                Commands.runOnce(innerIntakeSubsystem.stop, innerIntakeSubsystem),
                                Commands.runOnce(transportSubsystem.stop, transportSubsystem));
        };

        public static final Supplier<Command> groundCommand = () -> Commands
                        .sequence(elevatorAndTiltAtPositionCommand.get(), intakeCommand.get())
                        .until(notedLoadedSubsystem.hasNote)
                        .andThen(stop.get());

        public static final Supplier<Command> groundCommandThenLevel = () -> groundCommand.get()
                        .until(RobotContainer.notedLoadedSubsystem.hasNote)
                        .andThen(Level.command.get());
}