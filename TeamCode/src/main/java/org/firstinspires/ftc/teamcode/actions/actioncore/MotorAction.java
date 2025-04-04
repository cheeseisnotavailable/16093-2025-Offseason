package org.firstinspires.ftc.teamcode.actions.actioncore;

import com.acmerobotics.dashboard.RobotStatus;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SuperStructure;

@Config
public abstract class MotorAction extends Action {
    protected int toleranceRange = 200;
    protected int finishRange = 15;
    protected SuperStructure upper;
    //Params not in super class
    protected int target;
    protected double power = 1;
    protected long timeOnStart;

    public MotorAction(SuperStructure upper, int target){
        this.upper = upper;
        this.target = target;
        timeOnStart = System.currentTimeMillis();
    }

    public MotorAction(SuperStructure upper, int target, int toleranceRange){
        this.upper = upper;
        this.target = target;
        this.toleranceRange = toleranceRange;
        timeOnStart = System.currentTimeMillis();
    }

    public MotorAction(SuperStructure upper, int target, int toleranceRange, double power){
        this.upper = upper;
        this.target = target;
        this.toleranceRange = toleranceRange;
        this.power = power;
        timeOnStart = System.currentTimeMillis();
    }

    public int getError() {
        return target - upper.getArmPosition();
    }

    public boolean canStartNext(){
        if((Math.abs(getError()) < toleranceRange)){
            return true;
        }else{
            return false;
        }
    }

    public boolean isFinished(){
        if((Math.abs(getError()) < finishRange)){
            return true;
        }else{
            return false;
        }
    }

    public void actuate() {
    }

    public void stop(){
        RobotLog.d("----Stopped: "+this.toString());
    }

    //Functions not in super class
    public void forceStop(){
    }

    public String toString() {
        return returnType() + " Target " + this.target + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "MotorAction";
    }



}
