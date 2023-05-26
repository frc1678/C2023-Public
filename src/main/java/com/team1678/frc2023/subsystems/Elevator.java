package com.team1678.frc2023.subsystems;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.logger.Log;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Elevator extends ServoMotorSubsystem {

    public static Elevator mInstance;

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private Elevator() {
        super(Constants.ElevatorConstants.kElevatorConstants);
        setNeutralBrake(true);
    }

    // Homing refers to moving the elevator into it's "zero" position. 
    private boolean mHoming = true;
    private boolean mNeedsToHome = true;
    private final DelayedBoolean mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.2); 

    public void setNeutralBrake(boolean brake) {
        NeutralMode wantedMode = brake ? NeutralMode.Brake : NeutralMode.Coast;
        mMaster.setNeutralMode(wantedMode);
        for (TalonFX talonFX : mSlaves) {
            talonFX.setNeutralMode(wantedMode);
        }
    }
    
    public void setWantHome(boolean home) {
        mHoming = home;
        // once homing is started, no longer needs to home
        if (mHoming) {
            mNeedsToHome = false;
        }
        // force update state
        writePeriodicOutputs();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledILooper) {
        mEnabledILooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setNeutralBrake(true);
                setSetpointMotionMagic(mConstants.kHomePosition);
            }

            @Override
            public void onLoop(double timestamp) {
                // constantly re-homing unless in open loop
                if (Util.epsilonEquals(getSetpointHomed(), mConstants.kHomePosition, .02)
                        && atHomingLocation() && mNeedsToHome) {
                    setWantHome(true);
                } else if (mControlState != ControlState.OPEN_LOOP) {
                    setWantHome(false);
                }
            }

            @Override
            public void onStop(double timestamp) {
                setNeutralBrake(true);
            }
        });
    }

    public Request elevatorRequest(double length, boolean waitForPosition) {
        return new Request() {

            @Override
            public void act() {
                setSetpointMotionMagic(length);
            }

            @Override
            public boolean isFinished() {
                return !waitForPosition || Util.epsilonEquals(mPeriodicIO.position_units, length, 0.1);
            }

        };
    }
    
    public Request elevatorWaitRequest(double length) {
        return new Request() {
            @Override 
            public void act() {

            }

            @Override 
            public boolean isFinished() {
                return Util.epsilonEquals(mPeriodicIO.position_units, length, 0.2);
            }
        };
    }

    public Request elevatorTuckWaitRequest(double length) {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return mPeriodicIO.position_units < length;
            }
        };
    }

    public Request elevatorExtendWaitRequest(double length) {
        return new Request() {
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return mPeriodicIO.position_units > length;
            }
        };
    }

    @Override
    public synchronized void setSetpointMotionMagic(double units) {
        if (units != mConstants.kHomePosition) {
            mNeedsToHome = true;
        }
        super.setSetpointMotionMagic(units);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.overrideSoftLimitsEnable(false);
        } else {
            mMaster.overrideSoftLimitsEnable(true);
        }

        if (mHoming) {
            setOpenLoop(-6.0 / mConstants.kMaxVoltage);
            if (mHomingDelay.update(Timer.getFPGATimestamp(),
                    Util.epsilonEquals(getVelocity(), 0.0, 0.01))) {
                zeroSensors();  
                setSetpointMotionMagic(mConstants.kHomePosition);
                mHoming = false;
            }
            super.writePeriodicOutputs();
        } else {
            super.writePeriodicOutputs();
        }
    }
    
    @Override
    public boolean atHomingLocation() {
        return mPeriodicIO.position_units < mConstants.kHomePosition
                || Util.epsilonEquals(mPeriodicIO.position_units, mConstants.kHomePosition, 0.05);
    }

    @Log
    public double getElevatorAngleUnits(){
        return mPeriodicIO.position_units;
    }
    
    @Log
    public double getElevatorAngleTicks(){
        return mPeriodicIO.position_ticks;
    }

    @Log
    public double getElevatorSetpoints(){
        return getSetpointHomed();
    }
    
    @Log
    public double getElevatorDemand(){
        return mPeriodicIO.demand;
    }
    
    @Log
    public double getElevatorVelocity(){
        return mPeriodicIO.velocity_ticks_per_100ms;
    }
    
    @Log
    public double getElevatorVolts(){
        return mPeriodicIO.output_voltage;
    }
    
    @Log
    public double getElevatorCurrent(){
        return mPeriodicIO.master_current;
    }
    
    @Log
    public boolean getElevatorHoming(){
        return mHoming;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getBusVoltage();
    }

    @Log
    public double getFollowerMotorBusVolts() {
        return mSlaves[0].getBusVoltage();
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Position Units", mPeriodicIO.position_units);
        SmartDashboard.putNumber(mConstants.kName + " Position Ticks", mPeriodicIO.position_ticks);
        SmartDashboard.putNumber(mConstants.kName + " Demand", getSetpointHomed());
        SmartDashboard.putNumber(mConstants.kName + " Velocity", mPeriodicIO.velocity_ticks_per_100ms);
        SmartDashboard.putNumber(mConstants.kName + " Trajectory Velocity", mPeriodicIO.active_trajectory_velocity);
        SmartDashboard.putNumber(mConstants.kName + " Output Volts", mPeriodicIO.output_voltage);
        SmartDashboard.putNumber(mConstants.kName + " Current", mPeriodicIO.master_current);
        SmartDashboard.putBoolean(mConstants.kName + " Homing", mHoming);
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
