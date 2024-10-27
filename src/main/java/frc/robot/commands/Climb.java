package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Climb {
        private static final class Constants {
                private static final Measure<Distance> elevatorHeightStart = Meters.of(0.37);
                private static final Measure<Distance> elevatorHeightEnd = Meters.of(0.15);
                private static final Measure<Angle> tiltAngle = Degrees.of(0.0);
        }

        public static final Supplier<Command> createClimbPrep = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorHeightStart),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle));
                command.setName("Climb Prep");
                return command;
        };

        public static final Supplier<Command> createClimbEnd = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorHeightEnd),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle));
                command.setName("Climb End");
                return command;
        };
}