package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public abstract class BabyBird {
        private static final class Constants {
                private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(55, Degrees);
                private static MutableMeasure<Distance> elevatorPosition = MutableMeasure.ofBaseUnits(0.0,
                                Meters);
                private static double topShooterPercent = 0.2;
                private static double bottomShooterPercent = -0.2;
                private static double singulatorPercent = 0.5;
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
                                BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent));
        };

        private static final Supplier<Command> babyBirdCommand = () -> Commands
                        .sequence(elevatorAndTiltAtPositionCommand.get(), intakeCommand.get());

        public static final Supplier<Command> babyBirdCommandThenLevel = () -> babyBirdCommand.get()
                        .until(RobotContainer.notedLoadedSubsystem.hasNoteBB)
                        .andThen(Level.command.get());
}