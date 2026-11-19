package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Calibração automática do PIDF.
 * Antes de iniciar o PID, se pressionar B, executa um teste a 100% de potência para medir a velocidade máxima em RPM,
 * e calcula o valor teórico de F (F = 32767 / MaxTicksPerSec).
 */
@Config
@Configurable
@TeleOp(name = "PIDTuningnew")
public class PIDTuning extends LinearOpMode {

    public static String MOTOR_NAME = "shooter";
    public static boolean REVERSE_MOTOR = false;
    public static double TICKS_PER_REV = 28.0;

    public static double kp = 10.0;
    public static double ki = 3.0;
    public static double kd = 0.0;
    public static double f = 12.0; // Valor inicial antes do teste

    public static double targetRPM = 1000.0;
    public static double measuredMaxRPM = 0.0;
    public static double measuredMaxTicksPerSec = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        if (REVERSE_MOTOR) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorEx.Direction.FORWARD);
        }

        telemetry.addData("Status", "Inicializado!");
        telemetry.addLine("Pressione B durante a inicialização para calibrar o F, ou aguarde o PID.");
        telemetry.update();

        waitForStart();

        // ETAPA 1: CALIBRAÇÃO SE B ESTIVER PRESSIONADO
        if (gamepad1.b) {
            telemetry.addLine("=== ETAPA 1: CALIBRAÇÃO DO FEEDFORWARD (F) ===");
            telemetry.addLine("Acelerando o motor a 100% de potência para medir a velocidade máxima...");
            telemetry.update();

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1.0);

            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            // Aguarda 500ms para vencer a inércia
            while (opModeIsActive() && timer.milliseconds() < 500) {
                sleep(10);
            }

            // Amostra durante 5 segundos para medir a velocidade máxima
            double maxVelocitySample = 0.0;
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 5) {
                double currentVel = Math.abs(motor.getVelocity());
                if (currentVel > maxVelocitySample) {
                    maxVelocitySample = currentVel;
                }
                double currentRPM = (currentVel * 60.0) / TICKS_PER_REV;
                telemetry.addData("Calibrando - RPM Atual", "%.2f", currentRPM);
                telemetry.addData("Calibrando - Ticks/s Atual", "%.2f", currentVel);
                telemetry.update();
                sleep(20);
            }

            // Desliga o motor para troca de modo
            motor.setPower(0.0);

            measuredMaxTicksPerSec = maxVelocitySample;
            measuredMaxRPM = (measuredMaxTicksPerSec * 60.0) / TICKS_PER_REV;

            // Fórmula da Documentação REV: F = 32767 / maxVelocityInTicksPerSecond
            if (measuredMaxTicksPerSec > 0) {
                f = 32767.0 / measuredMaxTicksPerSec;
            }

            // Aguarda o usuário pressionar A para iniciar o PIDF (COM TELEMETRIA ATIVA)
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addLine("=== CALIBRAÇÃO DE F CONCLUÍDA ===");
                telemetry.addData("RPM Máximo Medido (100% Power)", "%.2f RPM", measuredMaxRPM);
                telemetry.addData("Ticks/s Máximo Medido", "%.2f", measuredMaxTicksPerSec);
                telemetry.addData("F Calculado (32767 / MaxTicks/s)", "%.4f", f);
                telemetry.addLine("");
                telemetry.addLine("--> Espere o motor parar e PRESSIONE 'A' PARA INICIAR O PIDF <--");
                telemetry.update(); // CRÍTICO: Atualiza a tela a cada ciclo!
                sleep(50);
            }
        }

        // ETAPA 2: CONFIGURAÇÃO DO MODO ENCODER E APLICAÇÃO DO PIDF
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Aplica o F juntamente com os ganhos kp, ki, kd
        motor.setVelocityPIDFCoefficients(kp, ki, kd, f);

        PIDFCoefficients lastPIDF = new PIDFCoefficients(kp, ki, kd, f);

        // ETAPA 3: LAÇO PRINCIPAL DE SINTONIA PIDF
        while (opModeIsActive()) {

            // Atualiza os parâmetros caso alterados pelo Dashboard
            PanelsConfigurables.INSTANCE.refreshClass(this);

            // Atualiza os coeficientes no motor em tempo real apenas se forem alterados
            if (lastPIDF.p != kp || lastPIDF.i != ki || lastPIDF.d != kd || lastPIDF.f != f) {
                motor.setVelocityPIDFCoefficients(kp, ki, kd, f);
                lastPIDF = new PIDFCoefficients(kp, ki, kd, f);
            }

            // Inversão dinâmica de sentido
            if (REVERSE_MOTOR && motor.getDirection() != DcMotorEx.Direction.REVERSE) {
                motor.setDirection(DcMotorEx.Direction.REVERSE);
            } else if (!REVERSE_MOTOR && motor.getDirection() != DcMotorEx.Direction.FORWARD) {
                motor.setDirection(DcMotorEx.Direction.FORWARD);
            }

            // Converte RPM desejado em Ticks por Segundo
            double targetTicksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;

            // Define a velocidade no PIDF Nativo
            motor.setVelocity(targetTicksPerSec);

            // Medições do encoder
            double currentTicksPerSec = motor.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / TICKS_PER_REV;
            double errorRPM = targetRPM - currentRPM;

            // Telemetria do Laço Principal (Driver Station + FTC Dashboard)
            telemetry.addLine("=== LAÇO PIDF EM EXECUÇÃO ===");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Error RPM", errorRPM);

            telemetry.addData("Target Ticks/s", targetTicksPerSec);
            telemetry.addData("Current Ticks/s", currentTicksPerSec);

            telemetry.addData("Max Measured RPM (100% Power)", measuredMaxRPM);
            telemetry.addData("Kp (REV)", kp);
            telemetry.addData("Ki (REV)", ki);
            telemetry.addData("Kd (REV)", kd);
            telemetry.addData("F (REV Calculado)", f);
            telemetry.addData("Potência Aplicada", "%.2f", motor.getPower());

            telemetry.update();
        }
    }
}
