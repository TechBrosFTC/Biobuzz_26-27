package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TurretCaalibrating extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo torreta;
        CRServo torreta1;
        DcMotorEx encoder;

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        torreta = hardwareMap.get(CRServo.class, "torreta");
        torreta1 = hardwareMap.get(CRServo.class, "torreta1");

        torreta1.setDirection(DcMotorSimple.Direction.FORWARD);
        torreta.setDirection(DcMotorSimple.Direction.REVERSE);


        double power = 0;
        waitForStart();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double ini_position = encoder.getCurrentPosition();

        sleep(500);

        while (opModeIsActive() && Math.abs(encoder.getCurrentPosition()) < 3){
            power += 0.01;
            torreta1.setPower(power);
            torreta.setPower(power);
            sleep(500);
        }
        torreta1.setPower(0);
        torreta.setPower(0);

        while (opModeIsActive()){
            telemetry.addData("power: ", power);
            telemetry.update();
        }
    }
}
