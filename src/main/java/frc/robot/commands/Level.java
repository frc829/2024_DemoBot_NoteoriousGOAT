package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Level {
        private static final class Constants {
                private static final Measure<Angle> tiltAngle = Degrees.of(0.0);
                private static final Measure<Distance> elevatorPosition = Meters.of(0.0);
                public static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                        return BasicCommands.Elevator.createAtPositionCondition.apply(elevatorPosition)
                                        .getAsBoolean() &&
                                        BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                                                        .getAsBoolean();

                };

        }

        public static final Supplier<Command> command = () -> {
                return Commands.parallel(
                                BasicCommands.Elevator.createSetElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.tiltAngle))
                                .until(Constants.elevatorAndTiltAtPositionCondition);
        };
}