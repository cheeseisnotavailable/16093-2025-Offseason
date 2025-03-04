package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.MotorAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

@Config
public class SlideAction extends MotorAction {

    public SlideAction(SuperStructure upper, slideState state){
        super(upper, 0);
        this.target = returnSlideTarget(state);
    }

    public SlideAction(SuperStructure upper, slideState state, int toleranceRange){
        super(upper, 0, toleranceRange);
        this.target = returnSlideTarget(state);
    }

    public SlideAction(SuperStructure upper, slideState state, int toleranceRange, double power){
        super(upper, 0, toleranceRange, power);
        this.target = returnSlideTarget(state);
    }

    public int getError() {
        return target - upper.getSlidesPosition();
    }

    public void actuate() {
        upper.setSlidesByP(target,power);
    }

    public void stop(){
        upper.setSlidePower(0);
        toleranceRange = 10000;
        super.stop();
    }

    //Functions not in super class
    public void forceStop(){
        upper.setSlidePower(0);
        toleranceRange = 10000;
        finishRange = 10000;
//        Action.actions.remove(this);
    }

    public String toString() {
        return returnType() + " Target " + this.target + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "SlideAction";
    }

    public enum slideState{
        SLIDE_MIN,
        SLIDE_HOLD_ASCENT,
        SLIDE_OPENLOOP_LIMIT,
        SLIDE_SLIGHTLY_LONGER,
        SLIDE_LONGER,
        SLIDE_LOW_BASKET,
        SLIDE_INTAKE_NEAR,
        SLIDE_INTAKE_WALL_SPECIMEN,
        SLIDE_AUTO_INTAKE_LAST_BLUE,
        SLIDE_AUTO_INTAKE_LAST_RED,
        SLIDE_AUTO_INTAKE_FIRST,
        SLIDE_AUTO_INTAKE_YELLOW,
        SLIDE_SWITCH_LIMIT,
        SLIDE_INTAKE_FAR,
        SLIDE_HIGH_CHAMBER_AIM_AUTO,
        SLIDE_HIGH_CHAMBER_AIM_TELEOP,
        SLIDE_HIGH_CHAMBER_PLACE,
        SLIDE_HIGH_CHAMBER_PLACE_AUTO,
        SLIDE_ASCENT_UP,
        SLIDE_ASCENT_DOWN,
        SLIDE_MAX
    }

    private int returnSlideTarget(slideState state){
        switch(state){
            case SLIDE_MIN:
                return SSValues.SLIDE_MIN;
            case SLIDE_HOLD_ASCENT:
                return SSValues.SLIDE_HOLD_ASCENT;
            case SLIDE_OPENLOOP_LIMIT:
                return SSValues.SLIDE_OPENLOOP_LIMIT;
            case SLIDE_SLIGHTLY_LONGER:
                return SSValues.SLIDE_SLIGHTLY_LONGER;
            case SLIDE_LONGER:
                return SSValues.SLIDE_LONGER;
            case SLIDE_LOW_BASKET:
                return SSValues.SLIDE_LOW_BASKET;
            case SLIDE_INTAKE_NEAR:
                return SSValues.SLIDE_INTAKE_NEAR;
            case SLIDE_INTAKE_WALL_SPECIMEN:
                return SSValues.SLIDE_INTAKE_WALL_SPECIMEN;
            case SLIDE_AUTO_INTAKE_LAST_BLUE:
                    return SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE;
            case SLIDE_AUTO_INTAKE_LAST_RED:
                return SSValues.SLIDE_AUTO_INTAKE_LAST_RED;
            case SLIDE_AUTO_INTAKE_FIRST:
                return SSValues.SLIDE_AUTO_INTAKE_FIRST;
            case SLIDE_AUTO_INTAKE_YELLOW:
                return SSValues.SLIDE_AUTO_INTAKE_YELLOW;
            case SLIDE_SWITCH_LIMIT:
                return SSValues.SLIDE_SWITCH_LIMIT;
            case SLIDE_INTAKE_FAR:
                return SSValues.SLIDE_INTAKE_FAR;
            case SLIDE_HIGH_CHAMBER_AIM_AUTO:
                return SSValues. SLIDE_HIGH_CHAMBER_AIM_AUTO;
            case SLIDE_HIGH_CHAMBER_AIM_TELEOP:
                return SSValues.SLIDE_HIGH_CHAMBER_AIM_TELEOP;
            case SLIDE_HIGH_CHAMBER_PLACE:
                return SSValues.SLIDE_HIGH_CHAMBER_PLACE;
            case SLIDE_HIGH_CHAMBER_PLACE_AUTO:
                return SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO;
            case SLIDE_ASCENT_UP:
                return SSValues.SLIDE_ASCENT_DOWN;
            case SLIDE_MAX:
                return SSValues.SLIDE_MAX;
        }
        return 0;
    }

}
