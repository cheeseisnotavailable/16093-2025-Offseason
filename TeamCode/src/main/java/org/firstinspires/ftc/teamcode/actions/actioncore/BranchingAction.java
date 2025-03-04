package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

import java.util.function.BooleanSupplier;

public class BranchingAction extends Action {
    private BranchActionPair[] branchActionPairs;
    boolean canStartNext = false;
    Runnable endWithThisRunnable;


    public BranchingAction(BranchActionPair...branchActionPairs){
        this.branchActionPairs = branchActionPairs;
    }


    public boolean canStartNext(){
        return isFinished();
    }

    public boolean isFinished(){
        for(BranchActionPair b:branchActionPairs){
            if(b.pairedAction.canStartNext()){
                endWithThisRunnable = b.pairedRunnable;
                return true;
            }
        }
        return false;
    }

    public void actuate() {
        for(BranchActionPair b:branchActionPairs){
            b.pairedAction.actuate();
        }
    }

    public void stop(){
        for(BranchActionPair b:branchActionPairs){
            b.pairedAction.stop();
        }
        endWithThisRunnable.run();
    }

    public void forceStop(){
        for(BranchActionPair b:branchActionPairs){
            b.pairedAction.forceStop();
        }
    }

    public String toString() {
        return returnType();
    }

    public String returnType(){
        return "BranchingAction";
    }

}
