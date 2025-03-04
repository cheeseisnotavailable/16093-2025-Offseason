package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

/**
 * This is essentially a mini buildSequence so that you can put Actions in sequence in
 * places where only one Action is supposed to be
 */

public class SequentialActionGroup extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private Action[] sequentialActions;
    private Action parent = new Action();
    private Action currentAction;
    private Runnable update;

    public SequentialActionGroup(Runnable update, Action...sequentialActions){
        this.update = update;
        this.sequentialActions = sequentialActions;
    }

    public SequentialActionGroup( Action parent, Runnable update, Action...sequentialActions){
        this.parent = parent;
        this.update = update;
        this.sequentialActions = sequentialActions;
    }

    public int getError() {
        int sum = 0;
        for(Action a: sequentialActions){
            sum += a.getError();
        }
        return sum/ sequentialActions.length;
    }

    public boolean canStartNext(){
        boolean canStart = true;
        for(Action a: sequentialActions){
            canStart = a.canStartNext() && canStart;
        }
        return canStart;
    }

    public boolean isFinished(){
        boolean canFinish = true;
        for(Action a: sequentialActions){
            canFinish = a.isFinished() && canFinish;
        }
        return canFinish;
    }

    public void actuate() {
        for (int i = 0; i < sequentialActions.length; i++) {
            currentAction = sequentialActions[i];
            currentAction.actuate();

            while(!currentAction.canStartNext()){
                update.run();

                if(parent.isFinished()){
                    this.stop();
                }

                if(currentAction.isFinished()){
                    currentAction.stop();
                }
            }
        }
    }

    public void stop(){
        for(Action a: sequentialActions){
            a.stop();
        }
    }

    public void forceStop(){
        for(Action a: sequentialActions){
            a.forceStop();
        }
    }

    public String toString(){
        return returnType()+" "+ currentAction.returnType();
    }

    public String returnType(){
        return "SequentialAction";
    }

}
