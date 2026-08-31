package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teste Intake & Rampa")
public class testemotor extends LinearOpMode {

    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor rampa;
    
    private double forcaGeral = 0.5;
    private boolean anteriorRT = false;
    private boolean anteriorLT = false;

    @Override
    public void runOpMode() {
        // Inicializa os motores
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        rampa = hardwareMap.get(DcMotor.class, "rampa");

        telemetry.addData("Status", "Aguardando início...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            // --- AJUSTE DE FORÇA PELOS GATILHOS (0.1 em 0.1) ---
            boolean atualRT = gamepad1.right_trigger > 0.5;
            boolean atualLT = gamepad1.left_trigger > 0.5;

            if (atualRT && !anteriorRT) {
                forcaGeral += 0.1;
            }
            if (atualLT && !anteriorLT) {
                forcaGeral -= 0.1;
            }
            
            forcaGeral = Range.clip(forcaGeral, 0.0, 1.0);
            anteriorRT = atualRT;
            anteriorLT = atualLT;

            // --- CONTROLE DO INTAKE SIMULTÂNEO (A/B) ---
            if (gamepad1.a) {
                // Sentidos opostos
                intake1.setPower(forcaGeral);
                intake2.setPower(-forcaGeral);
                telemetry.addData("Intake", "Coletando (A)");
            } else if (gamepad1.b) {
                // Sentidos opostos invertidos
                intake1.setPower(-forcaGeral);
                intake2.setPower(forcaGeral);
                telemetry.addData("Intake", "Expulsando (B)");
            } else {
                intake1.setPower(0.0);
                intake2.setPower(0.0);
                telemetry.addData("Intake", "Parado");
            }

            // --- CONTROLE DO MOTOR RAMPA (X/Y) ---
            if (gamepad1.y) {
                rampa.setPower(forcaGeral);
                telemetry.addData("Rampa", "Sentido Horário (Y)");
            } else if (gamepad1.x) {
                rampa.setPower(-forcaGeral);
                telemetry.addData("Rampa", "Sentido Anti-horário (X)");
            } else {
                rampa.setPower(0.0);
                telemetry.addData("Rampa", "Parado");
            }

            telemetry.addData("Força Configurada", "%.2f", forcaGeral);
            telemetry.update();
        }
    }
}
