package com.compLevel1;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;
import java.util.function.Consumer;
import java.util.function.Function;

import com.compLevel0.Motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class Spinner {
    public final Measure<Voltage> voltage;
    public final Measure<Dimensionless> velocity;
    public final Consumer<Double> spin;
    public final Runnable stop;
    public final Runnable update;

    private Spinner(
            Measure<Voltage> voltage,
            Measure<Dimensionless> velocity,
            Consumer<Double> spin,
            Runnable stop,
            Runnable update) {
        this.voltage = voltage;
        this.velocity = velocity;
        this.spin = spin;
        this.stop = stop;
        this.update = update;
    }

    public static final Function<Motor, Spinner> create = (motor) -> {
        MutableMeasure<Dimensionless> velocity = MutableMeasure.zero(Value);

        MutableMeasure<Velocity<Angle>> spinSetpoint = MutableMeasure.zero(RPM);
        Consumer<Double> spin = (setpoint) -> {
            spinSetpoint.mut_setMagnitude(motor.maxAngularVelocity.in(RPM));
            spinSetpoint.mut_times(setpoint);
            motor.spin.accept(spinSetpoint);
        };
        Runnable stop = () -> motor.stop.run();
        Runnable update = () -> {
            motor.update.run();
            velocity.mut_setMagnitude(motor.angularVelocity.in(RPM));
            velocity.mut_divide(motor.maxAngularVelocity.in(RPM));
        };

        return new Spinner(
                motor.voltage,
                velocity,
                spin,
                stop,
                update);
    };

}
