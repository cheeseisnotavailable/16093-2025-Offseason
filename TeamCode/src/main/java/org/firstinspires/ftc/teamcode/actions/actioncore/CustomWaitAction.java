package org.firstinspires.ftc.teamcode.actions.actioncore;

/**
 * A shortcut for anything else you might want to queue, with a timer as a finish condition
 */
public class CustomWaitAction extends Action {
    private int waitTime = 0;
    //Params not in super class
    private long timeOnStart;
    private Runnable doThis;

    public CustomWaitAction(Runnable doThis, int waitTime){
        this.waitTime = waitTime;
        this.doThis = doThis;
        timeOnStart = System.currentTimeMillis();
    }


    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > waitTime){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public boolean isFinished() {
        return canStartNext();
    }

    public void actuate(){
        doThis.run();
    }


    public String toString() {
        return returnType() + " Time " + this.waitTime;
    }

    public String returnType(){
        return "SequencerWaitAction";
    }
}
