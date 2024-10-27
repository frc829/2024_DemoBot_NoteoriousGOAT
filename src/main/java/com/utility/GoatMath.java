package com.utility;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static java.lang.Math.*;

public class GoatMath {

    private static final class Constants {
        private static final class NewtonRaphson {
            private static final double EPSILON = 0.001;
            private static final double maxIterationCount = 1000;
        }
    }

    private GoatMath() {
    }

    public static final double round(double value, int decimalPoints) {
        double sign = signum(value);
        value = abs(value);
        value *= pow(10, decimalPoints);
        value = floor(value);
        value /= pow(10, decimalPoints);
        return sign * value;

    }

    public static final double NewtonRaphsonSolver(
            Function<Double, Double> func,
            Function<Double, Double> deriv,
            double guess) {

        double h = func.apply(guess) / deriv.apply(guess);
        int iterations = 0;
        double time = Timer.getFPGATimestamp();
        while (abs(h) >= Constants.NewtonRaphson.EPSILON && iterations < Constants.NewtonRaphson.maxIterationCount) {
            h = func.apply(guess) / deriv.apply(guess);
            guess -= h;
            iterations++;
        }
        SmartDashboard.putNumber("NewtonRaphsonSolverTime", Timer.getFPGATimestamp() - time);
        return guess;
    }
}
