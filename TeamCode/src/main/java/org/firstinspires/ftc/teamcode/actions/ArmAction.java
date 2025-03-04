package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.MotorAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

@Config
public class ArmAction extends MotorAction {
    private int armTarget = 0;

    public ArmAction(SuperStructure upper, armState state){
        super(upper, 0);
        this.target = armTarget = returnArmTarget(state);

    }

    public ArmAction(SuperStructure upper, armState state, int toleranceRange){
        super(upper, 0, toleranceRange);
        this.target = armTarget = returnArmTarget(state);
    }

    public ArmAction(SuperStructure upper, armState state, int toleranceRange, double power){
        super(upper, 0, toleranceRange, power);
        this.target = armTarget = returnArmTarget(state);
    }

    public int getError() {
        return armTarget - upper.getArmPosition();
    }

    public void actuate() {
        upper.setArmByP(armTarget,power);
    }

    public void stop(){
        upper.setArmPower(0);
//        upper.armLimiter.reset(0);
        toleranceRange = 10000;
        super.stop();
    }

    //Functions not in super class
    public void forceStop(){
        upper.setArmPower(0);
//        upper.armLimiter.reset(0);
        toleranceRange = 10000;
        finishRange = 10000;
//        Action.actions.remove(this);
    }

    public String toString() {
        return returnType() + " Target " + this.armTarget + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "ArmAction";
    }

    public int returnArmTarget(armState state){
        int ret = 0;
        switch(state){
            case AUTO_ARM_OFFSET:
                ret = SSValues.AUTO_ARM_OFFSET;
                break;
            case ARM_DOWN:
                ret = SSValues.ARM_DOWN;
                break;
            case ARM_UP:
                ret = SSValues.ARM_UP;
                break;
            case ARM_HANG1:
                ret = SSValues.ARM_HANG1;
                break;
            case ARM_SLIGHTLY_HIGHER:
                ret = SSValues.ARM_SLIGHTLY_HIGHER;
                break;
            case ARM_ASCENT_AIM:
                ret = SSValues.ARM_ASCENT_AIM;
                break;
            case ARM_LOWER_FROM_BASKET:
                ret = SSValues.ARM_LOWER_FROM_BASKET;
                break;
            case ARM_ASCENT_SWING:
                ret = SSValues.ARM_ASCENT_SWING;
                break;
            case ARM_ASCENT_END:
                ret = SSValues.ARM_ASCENT_END;
                break;
            case ARM_GET_WALL_SPECIMEN:
                ret = SSValues.ARM_GET_WALL_SPECIMEN;
                break;
            case ARM_GET_WALL_SPECIMEN_UP:
                ret = SSValues.ARM_GET_WALL_SPECIMEN_UP;
                break;
        }
        return ret;
    }

    public enum armState{
        AUTO_ARM_OFFSET,
        ARM_DOWN,
        ARM_UP,
        ARM_HANG1,
        ARM_SLIGHTLY_HIGHER,
        ARM_ASCENT_AIM,
        ARM_LOWER_FROM_BASKET,
        ARM_ASCENT_SWING,
        ARM_ASCENT_END,
        ARM_GET_WALL_SPECIMEN,
        ARM_GET_WALL_SPECIMEN_UP
    }

}
