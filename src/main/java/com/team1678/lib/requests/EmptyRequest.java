package com.team1678.lib.requests;

/**
 * Empty request that does not execute any functions and will always return finished as true.
 */
public class EmptyRequest extends Request {

    @Override
    public void act() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}