package com.types;

/** Functional interface for a function that takes 6 inputs */
@FunctionalInterface
public interface SixFunction<In1, In2, In3, In4, In5, In6, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @param in4 Input 4
     * @param in5 Input 5
     * @param in6 Input 6
     * 
     * 
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3, In4 in4, In5 in5, In6 in6);
}