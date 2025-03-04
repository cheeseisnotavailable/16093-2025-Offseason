package org.firstinspires.ftc.teamcode.actions.actioncore;

import java.util.function.BooleanSupplier;

/**
 * A shortcut for anything else you might want to queue with a BooleanSupplier as a finish condition
 */
public class CustomFinishAction extends Action {
    //Params not in super class
    private BooleanSupplier bs;
    private Runnable doThis;

    public CustomFinishAction(Runnable doThis, BooleanSupplier bs){
        this.bs = bs;
        this.doThis = doThis;
    }

    public CustomFinishAction(BooleanSupplier bs){
        this.bs = bs;
        this.doThis = ()->{};
    }



    public boolean canStartNext(){
        return bs.getAsBoolean();
    }

    @Override
    public boolean isFinished() {
        return canStartNext();
    }

    public void actuate(){
        doThis.run();
    }


    public String toString() {
        return returnType() + " Supplier " + this.bs.toString();
    }

    public String returnType(){
        return "SequencerBooleanAction";
    }
}
