package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Amp {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(50);
                private static Measure<Distance> elevatorPosition = Meters.of(0.325);
                private static double topShooterPercent = -0.8;
                private static double bottomShooterPercent = -0.8;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.7;
        }

        public static final Supplier<Command> createAmpPosition = () -> {
                Command command = Commands.parallel(
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent),
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle));
                command.setName("Amp Position");
                return command;
        };

        public static final Supplier<Command> createAmpDrop = () -> {
                Command command = Commands.parallel(
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent),
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                                BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));

                command.setName("Amp Drop");
                return command;
        };
}