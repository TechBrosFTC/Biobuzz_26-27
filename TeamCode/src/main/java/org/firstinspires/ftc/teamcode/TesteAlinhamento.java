package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.opencv.core.Mat;

@TeleOp
public class TesteAlinhamento extends LinearOpMode {
    Limelight limelight;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new Limelight(hardwareMap);
        DriveTrain driveTrain = new DriveTrain(hardwareMap);

        double fator = 0.1;
        double kp = 0.2;
        telemetry.setMsTransmissionInterval(11);


        waitForStart();
        //22cm 35 graus
        while (opModeIsActive()){

            limelight.update();

            double erro = limelight.getTx();
            if(Math.abs(erro) > 2){
                driveTrain.curve((erro*kp)*fator);
            }else{
                driveTrain.parar();
            }
            telemetry.addData("tx: ", limelight.getTx());
            telemetry.addData("ty", limelight.getTy());
            telemetry.addData("distance", getDistance());
            telemetry.addData("tan:", Math.tan(Math.toRadians(65 + limelight.getTx())));
            telemetry.addData("valid:", limelight.isValid);
            telemetry.update();
        }

    }
    public double getDistance(){
        return (1.27 - 0.22) / Math.tan(Math.toRadians(55 + limelight.getTy()));
    }

}
