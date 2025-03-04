package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class TailAction extends ServoAction {

    public TailAction(SuperStructure upper, tailState state){
        super(upper, 0);
        this.pos = returnTailState(state);
    }

    public TailAction(SuperStructure upper, tailState state, int waitTime){
        super(upper, 0, waitTime);
        this.pos = returnTailState(state);
    }

    public void actuate() {
        upper.setTailPos(pos);
    }

    private double returnTailState(tailState state){
        switch(state){
            case TAIL_CHAMBER:
                return SSValues.TAIL_CHAMBER;
            case TAIL_DEFAULT:
                return SSValues.TAIL_DEFAULT;
            case TAIL_AUTO_POS:
                    return SSValues.TAIL_AUTO_POS;
        }
        return 0;
    }

    public enum tailState{
        TAIL_DEFAULT,
        TAIL_CHAMBER,
        TAIL_AUTO_POS
    }

}
