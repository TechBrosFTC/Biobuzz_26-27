package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private Limelight3A limelight;
    private LLResult result;
    private double tx = 0, ty = 0, ta = 0;
    public boolean isValid = false;
    public Limelight(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(0);
        this.limelight.setPollRateHz(90);
        limelight.start();
    }

    public void update(){
        result = limelight.getLatestResult();
        isValid = result.isValid();
        if(isValid){
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
        }else{
            tx = 0;
            ty = 0;
            ta = 0;
        }
    }
    public double getTx(){
        return tx;
    }
    public double getTy(){
        return ty;
    }
    public double getTa(){
        return ta;
    }

}



