package com.team1678.lib.requests;

import edu.wpi.first.wpilibj.Timer;

public class WaitRequest extends Request {

    private double waitTime = 0.0;
    private double startTime = 0.0;

    public WaitRequest(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void act() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= this.waitTime;
    }
    
}
