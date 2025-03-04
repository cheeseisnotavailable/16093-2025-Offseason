package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class IntakeAction extends ServoAction {

    public IntakeAction(SuperStructure upper, intakeState state){
        super(upper,0);
        this.pos = returnIntakePos(state);
    }

    public IntakeAction(SuperStructure upper, intakeState state, int waitTime){
        super(upper,0,waitTime);
        this.pos = returnIntakePos(state);
    }

    public String returnType(){
        return "IntakeAction";
    }

    public void actuate() {
        upper.setIntake(pos);
    }

    public enum intakeState{
        CONTINUOUS_SPIN,
        CONTINUOUS_STOP,
        CONTINUOUS_SPIN_OPPOSITE
    }

    public double returnIntakePos(intakeState state){
        switch(state){
            case CONTINUOUS_SPIN:
                return SSValues.CONTINUOUS_SPIN;
            case CONTINUOUS_STOP:
                return SSValues.CONTINUOUS_STOP;
            case CONTINUOUS_SPIN_OPPOSITE:
                return SSValues.CONTINUOUS_SPIN_OPPOSITE;
        }
        return 0;
    }

}
