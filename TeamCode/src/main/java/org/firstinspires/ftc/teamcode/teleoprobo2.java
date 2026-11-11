package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * OpMode TeleOp para controle exclusivo de movimentação da base Mecanum (Ateleop1),
 * sem nenhum dos subsistemas de mecanismos.
 */
@TeleOp(name = "teleoprobo2", group = "TeleOp")
public class teleoprobo2 extends LinearOpMode {

    // --- SENSORES E NAVEGAÇÃO / CHASSI ---
    private IMU imu;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    // --- GANHOS DO PID DE HEADING DA BASE ---
    double kp = 3;
    double kd = 4;
    double ki = 0.0006;

    double erro;
    double proporcional;
    double integral = 0;
    double ultimoErro = 0;
    double alvo = 0;

    double rx = 0;
    double multiplicador = 1;
    double curvapower = 1;

    String state = "parado";
    int index;
    ;

    // Direções discretas para translação
    private static final double[][] DIRECTIONS = {
            { 1,  0}, // direita
            { 1,  1}, // frente-direita
            { 0,  1}, // frente
            {-1,  1}, // frente-esquerda
            {-1,  0}, // esquerda
            {-1, -1}, // trás-esquerda
            { 0, -1}, // trás
            { 1, -1}  // trás-direita
    };

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicialização do IMU com orientação do Robot Controller
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(orientation));

        // Mapeamento dos motores do Chassi Mecanum
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        // Direções dos motores copiadas de Ateleop1
        direitaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData("Status", "Chassi Inicializado. Aguardando Start...");
        telemetry.update();

        waitForStart();

        double fator = 0.1;
        double kpLimelight = 0.2;

        while (opModeIsActive()) {

            // =======================================================
            // MOVIMENTAÇÃO DO CHASSI MECANUM (Ateleop1)
            // =======================================================
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double power = Math.min(Math.hypot(x, y), 1);
            double[] direction = getDirection(x, y);

            double moveX = direction[0] * power * multiplicador;
            double moveY = direction[1] * power * multiplicador;

            // Stick direito do Gamepad 1 controla a rotação da base
            double turnInput = gamepad1.right_stick_x;



            // --- 1. MODO GIRAR PURO (Totalmente independente, acionando motores puros sem mecanum) ---
            if (Math.abs(turnInput) > 0.05) {
                double turnPower = turnInput * curvapower;

                // Aplica potência diretamente nos motores (impede andar e girar ao mesmo tempo)
                esquerdaFrente.setPower(turnPower);
                esquerdaTras.setPower(turnPower);
                direitaFrente.setPower(-turnPower);
                direitaTras.setPower(-turnPower);

                alvo = gyro(); // Atualiza o rumo alvo constantemente
                integral = 0;
                ultimoErro = 0;
                state = "girando";
            }
            // --- 2. MODO TRANSLAÇÃO / PARADO (Usa a função Mecanum e PID de Rumo) ---
            else {
                if (moveX != 0 || moveY != 0) {
                    if (!state.equals("andando")) {
                        alvo = gyro();
                        integral = 0;
                        ultimoErro = 0;
                    }
                    rx = headingPID();
                    state = "andando";
                } else {
                    rx = 0;
                    integral = 0;
                    ultimoErro = 0;
                    state = "parado";
                }

                mecanum(moveX, moveY, rx + ((kpLimelight) * fator));
            }

            // =======================================================
            // TELEMETRIA DO CHASSI
            // =======================================================
            telemetry.addData("--- CHASSI MECANUM ---", "");
            telemetry.addData("Estado Base", state);
            telemetry.addData("Heading Gyro", "%.2f°", gyro());
            telemetry.addData("Alvo Gyro", "%.2f°", alvo);
            telemetry.addData("Multiplicador Translação", multiplicador);
            telemetry.addData("Curva Power Factor", curvapower);

            telemetry.addData("--- POTÊNCIA DOS MOTORES ---", "");
            telemetry.addData("Esquerda Frente (leftFront)", "%.2f", esquerdaFrente.getPower());
            telemetry.addData("Esquerda Trás (leftBack)", "%.2f", esquerdaTras.getPower());
            telemetry.addData("Direita Frente (rightFront)", "%.2f", direitaFrente.getPower());
            telemetry.addData("Direita Trás (rightBack)", "%.2f", direitaTras.getPower());

            telemetry.update();
        }
    }

    public double[] getDirection(double x, double y) {
        double deadband = 0.15;
        if (Math.hypot(x, y) < deadband) {
            index = -1;
            return new double[]{0, 0};
        }
        double angle = Math.toDegrees(Math.atan2(y, x));
        if (angle < 0) {
            angle += 360;
        }
        index = ((int)((angle + 22.5) / 45)) % 8;
        return DIRECTIONS[index];
    }

    public double headingPID() {
        erro = gyro() - alvo;
        proporcional = kp * erro;
        integral += erro;
        integral = Math.max(-100, Math.min(100, integral));
        double termoIntegral = integral * ki;
        double derivativa = (erro - ultimoErro) * kd;
        ultimoErro = erro;
        return (proporcional + termoIntegral + derivativa) / 100.0;
    }

    public void mecanum(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        esquerdaFrente.setPower(frontLeftPower);
        esquerdaTras.setPower(backLeftPower);
        direitaFrente.setPower(frontRightPower);
        direitaTras.setPower(backRightPower);
    }

    public double gyro() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
