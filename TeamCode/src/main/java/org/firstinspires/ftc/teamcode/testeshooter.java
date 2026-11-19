package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "testeshooter")
public class testeshooter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter, counterwheel;
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        counterwheel = hardwareMap.get(DcMotorEx.class, "counterWheel");
        waitForStart();
        double shot = 1;
        double countshot = 1;
        double velocity = 0; // Velocidade Alvo em RPM
        double ticksPerRev = 28.0; // Ticks por rotação do motor

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        counterwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(130, 0.12, 0, 12.7);
        counterwheel.setVelocityPIDFCoefficients(5.5, 0.36, 0, 36.4);

        while (opModeIsActive()){
            if(gamepad1.a){
                shot *= -1;
                sleep(100);
            }
            if(gamepad1.b){
                countshot *= -1;
                sleep(100);
            }
            if(gamepad1.right_bumper){
                velocity += 100;
                sleep(100);
            }
            if(gamepad1.left_bumper){
                velocity -= 100;
                sleep(100);
            }

            // Converte a velocidade de RPM para Ticks por segundo para o setVelocity()
            double shooterTicksPerSec = (velocity * shot * ticksPerRev) / 60.0;
            double counterTicksPerSec = (velocity * 2 * 0 * ticksPerRev) / 60.0;

            shooter.setVelocity(shooterTicksPerSec);
            counterwheel.setVelocity(counterTicksPerSec);

            // Leituras reais convertidas para RPM
            double currentShooterRPM = (shooter.getVelocity() * 60.0) / ticksPerRev;
            double currentCounterRPM = (counterwheel.getVelocity() * 60.0) / ticksPerRev;

            telemetry.addData("Target RPM:", velocity);
            telemetry.addData("Shooter Current RPM:", currentShooterRPM);
            telemetry.addData("Counterwheel Current RPM:", currentCounterRPM);
            telemetry.update();
        }

    }
}
