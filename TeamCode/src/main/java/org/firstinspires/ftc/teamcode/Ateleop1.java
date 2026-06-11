package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {

    private IMU imu;

    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    // Ganhos do PID
    double kp = 2.2;
    double kd = 2;
    double ki = 0.0006;

    // Variáveis do PID
    double erro;
    double proporcional;
    double integral = 0;
    double ultimoErro = 0;

    // Heading desejado
    double alvo = 0;

    // Saída do PID / rotação
    double rx = 0;

    // Multiplicadores
    double multiplicador = 0.8;
    double curvapower = 0.4;

    String state = "parado";

    int index;

    //           D      FD      F      FE      E      TE      T      TD
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

        imu = hardwareMap.get(IMU.class, "imu");

        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientation));

        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            // Intensidade do stick
            double power = Math.min(Math.hypot(x, y), 1);

            // Direção discreta
            double[] direction = getDirection(x, y);

            double moveX = direction[0] * power * multiplicador;
            double moveY = direction[1] * power * multiplicador;

            // Stick direito controla a curva
            double turnInput = gamepad1.right_stick_x;

            // Girando
            if (Math.abs(turnInput) > 0.1) {

                rx = turnInput * curvapower * multiplicador;

                // Atualiza continuamente o heading desejado
                alvo = gyro();

                integral = 0;
                ultimoErro = 0;

                state = "girando";
            }

            // Transladando
            else if (moveX != 0 || moveY != 0) {

                // Começou a andar
                if (!state.equals("andando")) {

                    alvo = gyro();

                    integral = 0;
                    ultimoErro = 0;
                }

                rx = headingPID();

                state = "andando";
            }

            // Parado
            else {

                rx = 0;

                integral = 0;
                ultimoErro = 0;

                state = "parado";
            }

            mecanum(moveX, moveY, rx);

            telemetry.addData("Heading", gyro());
            telemetry.addData("Alvo", alvo);
            telemetry.addData("Erro", erro);
            telemetry.addData("PID", rx);
            telemetry.addData("Setor", index);
            telemetry.addData("Estado", state);
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

        // Anti-windup
        integral = Math.max(-100, Math.min(100, integral));

        double termoIntegral = integral * ki;

        double derivativa = (erro - ultimoErro) * kd;

        ultimoErro = erro;

        return (proporcional + termoIntegral + derivativa) / 100.0;
    }

    public void mecanum(double x, double y, double rx) {

        double denominator = Math.max(
                Math.abs(y) +
                        Math.abs(x) +
                        Math.abs(rx),
                1);

        double frontLeftPower =
                (y + x + rx) / denominator;

        double backLeftPower =
                (y - x + rx) / denominator;

        double frontRightPower =
                (y - x - rx) / denominator;

        double backRightPower =
                (y + x - rx) / denominator;

        esquerdaFrente.setPower(frontLeftPower);
        esquerdaTras.setPower(backLeftPower);

        direitaFrente.setPower(frontRightPower);
        direitaTras.setPower(backRightPower);
    }

    public double gyro() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }
}