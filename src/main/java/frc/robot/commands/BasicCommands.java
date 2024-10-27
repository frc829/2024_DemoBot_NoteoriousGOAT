package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.RobotContainer.shooterTiltSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class BasicCommands {

    public static final class Elevator {
        public static final Measure<Distance> tolerance = Millimeters.of(5);
        public static final Measure<Distance> minPosition = Centimeters.of(0.0);
        public static final Measure<Distance> maxPosition = Centimeters.of(37);

        public static final Function<Measure<Distance>, BooleanSupplier> createAtPositionCondition = (position) -> {
            return () -> {
                return MathUtil.isNear(
                        position.in(Meters),
                        RobotContainer.elevatorSubsystem.position.in(Meters),
                        tolerance.in(Meters));
            };
        };

        public static final Function<DoubleSupplier, Runnable> driveElevator = (speedSupplier) -> {
            return () -> {
                if (RobotContainer.elevatorSubsystem.position.in(Meters) >= maxPosition.in(Meters)
                        && speedSupplier.getAsDouble() > 0) {
                    RobotContainer.elevatorSubsystem.drive.accept(0.0);

                } else if (RobotContainer.elevatorSubsystem.position.in(Meters) <= minPosition.in(Meters)
                        && speedSupplier.getAsDouble() < 0) {
                    RobotContainer.elevatorSubsystem.drive.accept(0.0);
                } else {
                    RobotContainer.elevatorSubsystem.drive.accept(speedSupplier.getAsDouble());
                }
            };
        };

        public static final Function<Measure<Distance>, Runnable> setElevatorPosition = (position) -> {
            return () -> RobotContainer.elevatorSubsystem.move.accept(position);
        };

        public static final Supplier<Command> createHoldElevatorCommand = () -> {
            Command command = Commands.run(
                    RobotContainer.elevatorSubsystem.hold,
                    RobotContainer.elevatorSubsystem);
            String name = String.format("Hold");
            command.setName(name);
            return command;
        };

        public static final Function<DoubleSupplier, Command> createDriveElevatorCommand = (speedSupplier) -> {
            Command command = Commands.run(
                    driveElevator.apply(speedSupplier),
                    RobotContainer.elevatorSubsystem);
            String name = String.format("Drive");
            command.setName(name);
            return command;
        };

        public static final Function<Measure<Distance>, Command> createSetAndHoldElevatorPositionCommand = (
                position) -> {
            Command command = Commands.run(
                    setElevatorPosition.apply(position),
                    RobotContainer.elevatorSubsystem);
            String name = String.format("Set And Hold at %s m", position.in(Meters));
            command.setName(name);
            return command;
        };

        public static final Function<Measure<Distance>, Command> createSetElevatorPositionCommand = (position) -> {
            Command command = Commands.run(
                    setElevatorPosition.apply(position),
                    RobotContainer.elevatorSubsystem);
            BooleanSupplier atToleranceCondition = createAtPositionCondition.apply(position);
            command = command.until(atToleranceCondition);
            String name = String.format("Set at %s m", position.in(Meters));
            command.setName(name);
            return command;
        };
    }

    public static final class Tilt {
        public static final Measure<Angle> tolerance = Degrees.of(0.5);
        public static final Measure<Angle> minAngle = Degree.of(-2);
        public static final Measure<Angle> maxAngle = Degree.of(58);

        public static final Function<Measure<Angle>, BooleanSupplier> createAtAngleCondition = (angle) -> {
            return () -> {
                return MathUtil.isNear(
                        angle.in(Degrees),
                        RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                        tolerance.in(Degrees));
            };
        };

        public static final Function<DoubleSupplier, Runnable> rotateTilt = (speedSupplier) -> {
            return () -> {
                if (RobotContainer.shooterTiltSubsystem.angle.in(Degrees) >= maxAngle.in(Degrees)
                        && speedSupplier.getAsDouble() > 0) {
                    RobotContainer.shooterTiltSubsystem.drive.accept(0.0);

                } else if (RobotContainer.shooterTiltSubsystem.angle.in(Degrees) <= minAngle.in(Degrees)
                        && speedSupplier.getAsDouble() < 0) {
                    RobotContainer.shooterTiltSubsystem.drive.accept(0.0);
                } else {
                    RobotContainer.shooterTiltSubsystem.drive.accept(speedSupplier.getAsDouble());
                }
            };
        };

        public static final Supplier<Runnable> tareRelativeEncoder = () -> {
            return () -> {
                shooterTiltSubsystem.setRelativeEncoder.accept(Degrees.of(0.0));
            };
        };

        public static final Function<Measure<Angle>, Runnable> setTiltAngle = (angle) -> {
            return () -> RobotContainer.shooterTiltSubsystem.turn.accept(angle);
        };

        public static final Supplier<Command> createHoldTiltCommand = () -> {
            Command command = Commands.run(
                    RobotContainer.shooterTiltSubsystem.hold,
                    RobotContainer.shooterTiltSubsystem);
            String name = String.format("Hold");
            command.setName(name);
            return command;
        };

        public static final Function<DoubleSupplier, Command> createRotateTiltCommand = (speedSupplier) -> {
            Command command = Commands.run(
                    rotateTilt.apply(speedSupplier),
                    RobotContainer.shooterTiltSubsystem);
            String name = String.format("Rotate");
            command.setName(name);
            return command;
        };

        public static final Function<Measure<Angle>, Command> createSetAndHoldTiltAngleCommand = (
                angle) -> {
            Command command = Commands.run(
                    setTiltAngle.apply(angle),
                    RobotContainer.shooterTiltSubsystem);
            String name = String.format("Set And Hold at %s deg", angle.in(Degrees));
            command.setName(name);
            return command;
        };

        public static final Function<Measure<Angle>, Command> createSetTiltAngleCommand = (angle) -> {
            Command command = Commands.run(
                    setTiltAngle.apply(angle),
                    RobotContainer.shooterTiltSubsystem);
            BooleanSupplier atToleranceCondition = createAtAngleCondition.apply(angle);
            command = command.until(atToleranceCondition);
            String name = String.format("Set at %s deg", angle.in(Degrees));
            command.setName(name);
            return command;
        };

        public static final Supplier<Command> createTareRelativeEncoderCommand = () -> {
            return Commands.runOnce(
                    tareRelativeEncoder.get(),
                    shooterTiltSubsystem);
        };
    }

    public static final class OuterIntake {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.outerIntakeSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.outerIntakeSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class InnerIntake {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.innerIntakeSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.innerIntakeSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class Transport {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.transportSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.transportSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class Singulator {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.singulatorSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.singulatorSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class TopShooter {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.topShooterSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.topShooterSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class BottomShooter {
        public static final Function<Double, Runnable> spin = (speed) -> {
            return () -> RobotContainer.bottomShooterSubsystem.spin.accept(speed);
        };
        public static final Function<Double, Command> createSpinCommand = (speed) -> {
            Command command = Commands.run(
                    spin.apply(speed),
                    RobotContainer.bottomShooterSubsystem);
            String name = String.format("Run at %s of Max", speed);
            command.setName(name);
            return command;
        };
    }

    public static final class ManualSpinners {
        public static final Runnable spin = () -> {
            double value = RobotContainer.operator.fullTriggerValue.getAsDouble();
            RobotContainer.topShooterSubsystem.spin.accept(value);
            RobotContainer.bottomShooterSubsystem.spin.accept(value);
            RobotContainer.singulatorSubsystem.spin.accept(value);
            RobotContainer.transportSubsystem.spin.accept(-value);
            RobotContainer.innerIntakeSubsystem.spin.accept(-value);
            RobotContainer.outerIntakeSubsystem.spin.accept(-value);
        };

        public static final Supplier<Command> spinCommand = () -> {
            Command command = Commands.run(
                    spin,
                    RobotContainer.bottomShooterSubsystem,
                    RobotContainer.topShooterSubsystem,
                    RobotContainer.innerIntakeSubsystem,
                    RobotContainer.outerIntakeSubsystem,
                    RobotContainer.transportSubsystem,
                    RobotContainer.singulatorSubsystem);
            String name = String.format("Run");
            command.setName(name);
            return command;
        };
    }

    public BasicCommands() {
    }

}
