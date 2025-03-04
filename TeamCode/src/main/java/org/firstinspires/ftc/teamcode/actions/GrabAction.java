package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class GrabAction extends ServoAction {

    public GrabAction(SuperStructure upper, grabState state){
        super(upper,0);
        this.pos = returnGrabPosition(state);
    }

    public GrabAction(SuperStructure upper, grabState state, int waitTime){
        super(upper,0,waitTime);
        this.pos = returnGrabPosition(state);
    }

    public String returnType(){
        return "GrabAction";
    }

    public void actuate() {
        upper.setGrabPos(pos);
    }

    public enum grabState{
        GRAB_DEFAULT,
        GRAB_OPEN,
        GRAB_CLOSED,
        GRAB_CLOSED_WITHOUT_CAP,
        AUTO_GRAB_CLOSED
    }

    public double returnGrabPosition(grabState state){
        switch (state) {
            case GRAB_DEFAULT:
                return SSValues.GRAB_DEFAULT;
            case GRAB_OPEN:
                return SSValues.GRAB_OPEN;
            case GRAB_CLOSED:
                return SSValues.GRAB_CLOSED;
            case GRAB_CLOSED_WITHOUT_CAP:
                return SSValues.GRAB_CLOSED_WITHOUT_CAP;
            case AUTO_GRAB_CLOSED:
                return SSValues.AUTO_GRAB_CLOSED;
        }
        return 0;
    }

}
