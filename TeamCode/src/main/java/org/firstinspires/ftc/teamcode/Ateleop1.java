package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;




@Config
@TeleOp(name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {

    // --- SENSORES E NAVEGAÇÃO / CHASSI ---
    private IMU imu;
    private Limelight limelight;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    private Servo servoRampa;

    // --- MOTORES DE MECANISMOS ---
    private DcMotorEx shooter;
    private DcMotorEx transfer;
    private DcMotorEx intake;
    private DcMotorEx passador;

    private GoBildaPinpointDriver odo;

    // --- SERVOS E TORRETA INSTANCIADA ---
    private Servo servorampa;
    private Servo servoPosicional;
    private CRServo torreta;
    private CRServo torreta1;
    private DcMotorEx encoder_torreta;
    private Turret turretObjeto;

    private Servo hood;

    // --- ALVO DE TRACKING DA TORRETA (60, 60 em Polegadas/MM) ---
    public static double TARGET_X = 60.0;
    public static double TARGET_Y = 60.0;

    // --- CONFIGURAÇÕES DE TRANSFER E INTAKE ---
    public static String TRANSFER_NAME = "transfer";
    public static String INTAKE_NAME = "intake";
    public static boolean REVERSE_INTAKE = false;

    // --- GANHOS PIDF TUNÁVEIS NO FTC DASHBOARD ---
    // Shooter PIDF
    public static double SHOOTER_P = 80;
    public static double SHOOTER_I = 0;

    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 13.4291;

    // Transfer PIDF
    public static double TRANSFER_P = 8;
    public static double TRANSFER_I = 0.0;
    public static double TRANSFER_D = 0.0;
    public static double TRANSFER_F = 12.41;

    // Passador PIDF
    public static double PASSADOR_P = 5.0;
    public static double PASSADOR_I = 0.0;
    public static double PASSADOR_D = 0.0;
    public static double PASSADOR_F = 12.0;

    // --- POSIÇÕES AJUSTÁVEIS DO SERVO POSICIONAL ---
    public static double POS_ABERTO = 1.0;
    public static double POS_FECHADO = 0.0;

    // --- CONTROLES DE BOTÃO E ESTADOS DA VARREDURA DA TORRETA ---
    private boolean abriuServoRampa = false;
    private boolean servoEstaAberto = false;
    private boolean anteriorX = false;

    private double searchDirection = -1.0;  // Sentido atual da busca da torreta entre os limites

    // --- GANHOS DO PID DE HEADING DA BASE ---
    double kp = 3;
    double kd = 0;
    double ki = 0;

    double erro;
    double proporcional;
    double integral = 0;
    double ultimoErro = 0;
    double alvo = 0;

    double rx = 0;
    double multiplicador = 0.65;
    double curvapower = 0.5;

    String state = "parado";
    int index;
    public static double incremento = 0;
    double shot = 1;
    double countshot = 1;
    public static int shooterVelocity = 2815;
    double shooterTicksPerRev = 28.0;
    public static int velinicial = 2815;
    public static double currentShooterRPM = 0;
    boolean atirando = false;

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime timer = new ElapsedTime();
        // Inicialização do IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));

        hood = hardwareMap.get(Servo.class, "hood");

        passador = hardwareMap.get(DcMotorEx.class, "passador");
        passador.setDirection(DcMotorSimple.Direction.FORWARD);
        passador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passador.setVelocityPIDFCoefficients(PASSADOR_P, PASSADOR_I, PASSADOR_D, PASSADOR_F);

        // Mapeamento dos motores do Chassi
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        direitaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        direitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.FORWARD);

        // Mapeamento e Inicialização dos motores do Teste Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Mapeamento de Transfer e Intake
        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setVelocityPIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);

        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        servorampa = hardwareMap.get(Servo.class, "servorampa");

        // --- INSTANCIAÇÃO DA CLASSE TURRET (SERVOS TORRETA E TORRETA1) ---
        torreta = hardwareMap.get(CRServo.class, "torreta");
        torreta1 = hardwareMap.get(CRServo.class, "torreta1");

        torreta1.setDirection(DcMotorSimple.Direction.REVERSE);
        torreta.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder_torreta = hardwareMap.get(DcMotorEx.class, "leftFront");

        double ticksPerDegreeTurret = 8192.0 / 36;
        turretObjeto = new Turret(torreta, torreta1, encoder_torreta, ticksPerDegreeTurret, DcMotorSimple.Direction.REVERSE);
        VoltageSensor voltagem;
        /*
         * Hardware do Pinpoint
         */
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderResolution(76.964, DistanceUnit.MM);
        odo.setOffsets(-130, -55, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        voltagem = hardwareMap.voltageSensor.iterator().next();
        // Inicializa o Pinpoint na Posição Inicial (10, 10, Heading 0)
        odo.initialize();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 10.0, 10.0, AngleUnit.DEGREES, 0.0));

        telemetry.addData("Status", "Inicializado com sucesso!");
        telemetry.update();

        limelight = new Limelight(hardwareMap);

        DriveTrain driveTrain = new DriveTrain(hardwareMap);

        double fator = 0.1;
        double kpLimelight = 0.2;
        telemetry.setMsTransmissionInterval(11);

        boolean passando = false;
        servorampa.setPosition(1);

        // =========================================================================
        // SINTONIA DO FEEDFORWARD (F) DO SHOOTER APÓS INIT E ANTES DE START
        // =========================================================================
        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.b) {
                telemetry.addLine("=== CALIBRAÇÃO DO FEEDFORWARD (F) DO SHOOTER ===");
                telemetry.addLine("Acelerando o motor do Shooter a 100% para medir velocidade máxima...");
                telemetry.update();

                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter.setPower(1.0);

                ElapsedTime calibTimer = new ElapsedTime();
                calibTimer.reset();

                // Aguarda 500ms para vencer a inércia
                while (!isStarted() && !isStopRequested() && calibTimer.milliseconds() < 500) {
                    sleep(10);
                }

                // Amostra a velocidade por 3 segundos para encontrar o valor máximo
                double maxVelTicks = 0.0;
                calibTimer.reset();
                while (!isStarted() && !isStopRequested() && calibTimer.seconds() < 3.0) {
                    double currentVel = Math.abs(shooter.getVelocity());
                    if (currentVel > maxVelTicks) {
                        maxVelTicks = currentVel;
                    }
                    double currentRPM = (currentVel * 60.0) / shooterTicksPerRev;
                    telemetry.addData("Calibrando Shooter - RPM Atual", "%.2f RPM", currentRPM);
                    telemetry.addData("Calibrando Shooter - Ticks/s Atual", "%.2f", currentVel);
                    telemetry.update();
                    sleep(20);
                }

                shooter.setPower(0.0);

                // Fórmula da Documentação REV / FTC: F = 32767 / maxVelocityInTicksPerSecond
                if (maxVelTicks > 0) {
                    SHOOTER_F = 32767.0 / maxVelTicks;
                }
                double maxRPMCalculado = (maxVelTicks * 60.0) / shooterTicksPerRev;

                while (!isStarted() && !isStopRequested() && !gamepad1.a) {
                    telemetry.addLine("=== CALIBRAÇÃO DE F DO SHOOTER CONCLUÍDA ===");
                    telemetry.addData("Shooter Max RPM Medido", "%.2f RPM", maxRPMCalculado);
                    telemetry.addData("Shooter Max Ticks/s", "%.2f", maxVelTicks);
                    telemetry.addData("SHOOTER_F Calculado (32767 / MaxTicks)", "%.4f", SHOOTER_F);
                    telemetry.addLine("");
                    telemetry.addLine("--> Espere o motor parar e PRESSIONE 'A' PARA CONFIRMAR <--");
                    telemetry.update();
                    sleep(50);
                }
            }

            telemetry.addData("Status", "Inicializado! Aguardando Start...");
            telemetry.addData("SHOOTER_F Configurado", "%.4f", SHOOTER_F);
            telemetry.addLine("Dica: Pressione 'B' no Gamepad 1 para calibrar o F do Shooter.");
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // Configura o motor para utilizar o PIDF com o F calibrado
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        while (opModeIsActive()) {
            // =======================================================
            // 1. PINPOINT ODOMETRY E RESET NO BOTÃO X (GAMEPAD 1)
            // =======================================================


            // =======================================================
            // 2. TRACKING E BUSCA AUTOMÁTICA DA TORRETA (PROCURA DE TAG)
            // =======================================================
            limelight.update();

            double powerTorretaManual = gamepad2.left_stick_x;

            if (Math.abs(powerTorretaManual) > 0.1) {
                // Override manual no Gamepad 2
                torreta.setPower(powerTorretaManual);
                torreta1.setPower(powerTorretaManual);
            } else if (limelight.isValid && Math.abs(limelight.getTx()) > 3 && !atirando && !passando) {
                // TAG ENCONTRADA: Rastreia com PID pelo caminho mais curto
                double targetTurretAngle = turretObjeto.getCurrentAngle() - limelight.getTx();
                turretObjeto.goToAngleShortestPath(targetTurretAngle);
                turretObjeto.update();
            } else {
                turretObjeto.update();
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double power = Math.min(Math.hypot(x, y), 1);
            double[] direction = getDirection(x, y);

            double moveX = direction[0] * power * multiplicador;
            double moveY = direction[1] * power * multiplicador;

            double turnInput = gamepad1.right_stick_x * curvapower;

            mecanum(moveX, moveY, rx + turnInput);


            if (gamepad2.right_bumper) {
                velinicial += 5;
                shooterVelocity = velinicial;
                sleep(100);
            }
            if (gamepad2.left_bumper) {
                shooterVelocity = velinicial;
                velinicial -= 5;
                sleep(100);
            }
            shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
            double shooterTicksPerSec = (shooterVelocity * shot * shooterTicksPerRev) / 60.0;
            shooter.setVelocity(shooterTicksPerSec);
            currentShooterRPM = (shooter.getVelocity() * 60.0) / shooterTicksPerRev;
            // Servo Transfer (Gamepad 1)


            // =======================================================
            // 5. MOTORES TRANSFER E INTAKE (GATILHOS RT / LT)
            // =======================================================
            double transferPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(transferPower) > 0.15) {
                transfer.setPower(transferPower);
                intake.setPower(transferPower);
            } else if(!passando && !atirando){
                transfer.setPower(0.0);
                intake.setPower(0.0);
            }

            if (gamepad1.right_bumper && !atirando) {
                if (currentShooterRPM >= shooterVelocity - 100) {
                    intake.setPower(0.6);
                    passador.setVelocity(convertRPMtoTicks(5500));
                    transfer.setVelocity(convertRPMtoTicks(5000));
                    timer.reset();
                    passando = true;
                }
            }
            if (passando) {
                if (transfer.getVelocity() > convertRPMtoTicks(1300)) {
                    atirando = true;
                    passando = false;
                    servorampa.setPosition(0.5);
                    transfer.setVelocity(convertRPMtoTicks(5000));
                    sleep(500);
                }
            }

            if (atirando) {
                shooterVelocity += incremento;
                shooter.setVelocity(convertRPMtoTicks(shooterVelocity));
                intake.setPower(1);
                if (timer.time() > 2.3) {
                    atirando = false;
                    timer.reset();
                    transfer.setVelocity(0.0);
                    passador.setVelocity(0);
                }
            }
            if (!atirando) {
                shooterVelocity = velinicial;
                servorampa.setPosition(1);
            }

            // =======================================================
            // 6. TELEMETRIA COMPLETA DE DADOS E ODOMETRIA
            // =======================================================
            double dist = getDistance(limelight.getTy());

            telemetry.addData("--- TORRETA TRACKING (60, 60) ---", "");
            telemetry.addData("Torreta Ângulo Atual", "%.2f°", turretObjeto.getCurrentAngle());
            telemetry.addData("Torreta Target Angle", "%.2f°", turretObjeto.getTargetAngle());
            telemetry.addData("Torreta Output PID", "%.2f", turretObjeto.getOutput());
            telemetry.addData("distancia", getDistance(limelight.getTy()));
            telemetry.addData("ty", limelight.getTy());
            telemetry.addData("--- SHOOTER (GAMEPAD 2) ---", "");
            telemetry.addData("Shooter Target RPM", shooterVelocity);
            telemetry.addData("Shooter Current RPM", "%.2f", currentShooterRPM);
            telemetry.addData("--- MECANISMOS ---", "");
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
        index = ((int) ((angle + 22.5) / 45)) % 8;
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

    public int convertRPMtoTicks(int rpm) {
        return rpm * 28 / 60;
    }

    public void mecanum(double x, double y, double rx) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        esquerdaFrente.setPower(-frontLeftPower);
        esquerdaTras.setPower(backLeftPower);
        direitaFrente.setPower(frontRightPower);
        direitaTras.setPower(backRightPower);
    }
    public double getDistance(double ty) {
        double targetHeight = 1.21;   // 1,21 metros do chão
        double cameraHeight = 0.412;  // 41,2 cm do chão (0,412 m)
        double cameraAngle = 48.0;    // 48 graus de inclinação da horizontal

        double angle = cameraAngle + ty;
        telemetry.addData("angle" , angle);
        telemetry.addData("target/camera: ", targetHeight-cameraHeight);
        telemetry.addData("tan", Math.tan(Math.toRadians(angle)));
        return (targetHeight - cameraHeight) * Math.tan(Math.toRadians(angle));
    }

    public double gyro() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
