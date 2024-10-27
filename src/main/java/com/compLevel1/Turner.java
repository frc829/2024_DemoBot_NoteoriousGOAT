package com.compLevel1;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
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

public class Turner {
    public final Measure<Voltage> voltage;
    public final Measure<Angle> angle;
    public final Measure<Angle> absoluteAngle;
    public final Measure<Dimensionless> velocity;
    public final Consumer<Measure<Angle>> setRelativeEncoder;
    public final Consumer<Measure<Angle>> turn;
    public final Consumer<Double> drive;
    public final Runnable resetRelEncoderFromAbsolute;
    public final Runnable hold;
    public final Runnable update;
    public final Runnable stop;

    private Turner(
            Measure<Voltage> voltage,
            Measure<Angle> angle,
            Measure<Angle> absoluteAngle,
            Measure<Dimensionless> velocity,
            Consumer<Measure<Angle>> setRelativeEncoder,
            Consumer<Measure<Angle>> turn,
            Consumer<Double> drive,
            Runnable resetRelEncoderFromAbsolute,
            Runnable hold,
            Runnable update,
            Runnable stop) {
        this.voltage = voltage;
        this.angle = angle;
        this.absoluteAngle = absoluteAngle;
        this.velocity = velocity;
        this.setRelativeEncoder = setRelativeEncoder;
        this.turn = turn;
        this.drive = drive;
        this.resetRelEncoderFromAbsolute = resetRelEncoderFromAbsolute;
        this.hold = hold;
        this.update = update;
        this.stop = stop;
    }

    public static final Function<Double, Function<Motor, Turner>> create = (
            gearing) -> (motor) -> {
                
                MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                MutableMeasure<Dimensionless> velocity = MutableMeasure.zero(Value);

                MutableMeasure<Angle> turnSetpoint = MutableMeasure.zero(Rotations);
                Consumer<Measure<Angle>> setRelativeEncoder = (setpoint) -> {
                    turnSetpoint.mut_setMagnitude(setpoint.in(Rotations));
                    turnSetpoint.mut_times(gearing);
                    motor.setRelativeEncoderAngle.accept(turnSetpoint);
                };
                Consumer<Measure<Angle>> turn = (setpoint) -> {
                    turnSetpoint.mut_setMagnitude(setpoint.in(Rotations));
                    turnSetpoint.mut_times(gearing);
                    motor.turn.accept(turnSetpoint);
                };
                MutableMeasure<Velocity<Angle>> driveSetpoint = MutableMeasure.zero(RPM);
                Consumer<Double> drive = (setpoint) -> {
                    driveSetpoint.mut_setMagnitude(motor.maxAngularVelocity.in(RPM));
                    driveSetpoint.mut_times(setpoint);
                    turnSetpoint.mut_setMagnitude(motor.angle.in(Rotations));
                    motor.spin.accept(driveSetpoint);
                };
                Runnable resetRelEncoderFromAbsolute = () -> {
                    double absoluteAngleValue = motor.absoluteAngle.in(Rotations);
                    if (absoluteAngleValue >= 0.5) {
                        absoluteAngleValue -= 1.0;
                    }
                    turnSetpoint.mut_setMagnitude(absoluteAngleValue * gearing);
                    motor.setRelativeEncoderAngle.accept(turnSetpoint);
                };
                Runnable hold = () -> motor.turn.accept(turnSetpoint);
                Runnable stop = () -> motor.stop.run();
                Runnable update = () -> {
                    motor.update.run();
                    angle.mut_setMagnitude(motor.angle.in(Rotations) / gearing);
                    double absoluteAngleValue = motor.absoluteAngle.in(Rotations);
                    if (absoluteAngleValue >= 0.5) {
                        absoluteAngleValue -= 1.0;
                    }
                    absoluteAngle.mut_setMagnitude(absoluteAngleValue);
                    velocity.mut_setMagnitude(motor.angularVelocity.in(RPM));
                    velocity.mut_divide(motor.maxAngularVelocity.in(RPM));
                };

                return new Turner(
                        motor.voltage,
                        angle,
                        absoluteAngle,
                        velocity,
                        setRelativeEncoder,
                        turn,
                        drive,
                        resetRelEncoderFromAbsolute,
                        hold,
                        update,
                        stop);
            };

}
