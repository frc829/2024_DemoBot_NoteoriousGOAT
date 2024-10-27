package com.compLevel1;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Value;
import java.util.function.Consumer;
import java.util.function.Function;

import com.compLevel0.Motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class Elevator {
    public final Measure<Voltage> voltage;
    public final Measure<Distance> position;
    public final Measure<Dimensionless> velocity;
    public final Consumer<Measure<Distance>> move;
    public final Consumer<Double> drive;
    public final Runnable hold;
    public final Runnable update;

    private Elevator(
            Measure<Voltage> voltage,
            Measure<Distance> position,
            Measure<Dimensionless> velocity,
            Consumer<Measure<Distance>> move,
            Consumer<Double> drive,
            Runnable hold,
            Runnable update) {
        this.voltage = voltage;
        this.position = position;
        this.velocity = velocity;
        this.move = move;
        this.drive = drive;
        this.hold = hold;
        this.update = update;
    }

    public static final Function<Double, Function<Measure<Distance>, Function<Motor, Elevator>>> create = (
            gearing) -> (drumRadius) -> (motor) -> {
                MutableMeasure<Distance> position = MutableMeasure.zero(Meters);
                MutableMeasure<Dimensionless> velocity = MutableMeasure.zero(Value);

                MutableMeasure<Angle> moveSetpoint = MutableMeasure.zero(Radians);
                Consumer<Measure<Distance>> move = (setpoint) -> {
                    moveSetpoint.mut_setMagnitude(setpoint.in(Meters));
                    moveSetpoint.mut_divide(drumRadius.in(Meters));
                    moveSetpoint.mut_times(gearing);
                    motor.turn.accept(moveSetpoint);
                };
                MutableMeasure<Velocity<Angle>> driveSetpoint = MutableMeasure.zero(RPM);
                Consumer<Double> drive = (setpoint) -> {
                    driveSetpoint.mut_setMagnitude(motor.maxAngularVelocity.in(RPM));
                    driveSetpoint.mut_times(setpoint);
                    moveSetpoint.mut_setMagnitude(motor.angle.in(Radians));
                    motor.spin.accept(driveSetpoint);
                };
                Runnable hold = () -> motor.turn.accept(moveSetpoint);
                Runnable update = () -> {
                    motor.update.run();
                    position.mut_setMagnitude(motor.angle.in(Radians) * drumRadius.in(Meters) / gearing);
                    velocity.mut_setMagnitude(motor.angularVelocity.in(RPM));
                    velocity.mut_divide(motor.maxAngularVelocity.in(RPM));
                };

                return new Elevator(
                        motor.voltage,
                        position,
                        velocity,
                        move,
                        drive,
                        hold,
                        update);
            };
}
