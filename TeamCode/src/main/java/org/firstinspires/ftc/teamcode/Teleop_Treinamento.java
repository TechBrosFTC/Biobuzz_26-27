package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * OpMode de Treinamento com suporte para calibração PIDF via FTC Dashboard (@Config)
 * e FullPanels (@Configurable) em tempo real via web (http://192.168.43.1:8080/dash).
 */
@Config
@Configurable
@TeleOp(name = "Teleop Treinamento", group = "Pedro Pathing")
public class Teleop_Treinamento extends LinearOpMode {

    private DcMotorEx motorTeste;

    public static double kp = 0.3;
    public static double ki = 0.00015;
    public static double kd = 0.0;
    public static double f = 11.1;

    public static double targetRPM = 1172.0;
    public static double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorTeste = hardwareMap.get(DcMotorEx.class, "motorTeste");

        motorTeste.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTeste.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Inicializado. Aguardando Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            motorTeste.setVelocityPIDFCoefficients(kp, ki, kd, f);

            double targetTicksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
            motorTeste.setVelocity(targetTicksPerSec);

            double currentTicksPerSec = motorTeste.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / TICKS_PER_REV;
            double errorRPM = targetRPM - currentRPM;

            telemetry.addData("Target RPM", "%.2f", targetRPM);
            telemetry.addData("Current RPM", "%.2f", currentRPM);
            telemetry.addData("Current Ticks/s", "%.2f", currentTicksPerSec);
            telemetry.addData("Error (RPM)", "%.2f", errorRPM);

            telemetry.addData("Kp", "%.6f", kp);
            telemetry.addData("Ki", "%.6f", ki);
            telemetry.addData("Kd", "%.6f", kd);
            telemetry.addData("F (Feedforward)", "%.6f", f);
            telemetry.addData("TICKS_PER_REV", "%.2f", TICKS_PER_REV);

            telemetry.addData("Potência Aplicada", "%.4f", motorTeste.getPower());
            telemetry.update();
        }
    }
}
