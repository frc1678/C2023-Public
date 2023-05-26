package com.team1678.lib.requests;

/**
 * A request that is able to call a method and run it without the need of building out the request infastructure
 * in the class. Helpful when testing out the functionality of subsystems.
 */
public class LambdaRequest extends Request {

    /**
     * Generates a method denoted by 'f' and is of the type void
     */
    public interface VoidInterface {
        void f();
    }

    /**
     * Makes a variable that holds a method 'f'
     */
    VoidInterface mF;

    /**
     * Setter for the vairable 'mF'
     * @param f Any function that is of class void
     */
    public LambdaRequest(VoidInterface f) {
        mF = f;
    }

    @Override
    public void act() {
        mF.f();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
