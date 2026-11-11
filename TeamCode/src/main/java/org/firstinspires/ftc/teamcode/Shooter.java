package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotorEx motor;
    private final Servo servo;
    private int targetRPM = 0;
    private int tolerance = 100;
    public Shooter(HardwareMap hardwareMap, Servo servo) {
        this.motor = hardwareMap.get(DcMotorEx.class, "shooter");
        this.servo = hardwareMap.get(Servo.class, "servo");
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void injectPolen(){
        servo.setPosition(1);
    }
    public void retractInjetor(){
        servo.setPosition(0);
    }


    public void setVelocity (double rpm){
        motor.setVelocity(convertRPMtoTicks(rpm));
    }
    public double getVelocity(){
        return  convertTickstoRPM(motor.getVelocity());
    }
    private double convertTickstoRPM(double ticks){
        return ticks*60/28;
    }
    private double convertRPMtoTicks(double ticks){
        return ticks*60/28;
    }
    public boolean isReady(){
        return getVelocity() <= targetRPM + tolerance && getVelocity() >= targetRPM - tolerance;
    }



}
