package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Timer;

@Autonomous (name = "Auto")
public class Auto extends LinearOpMode {

    public static boolean enableLimelightAim = false;

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    Thread turret = new Thread(new Runnable() {
        @Override
        public void run() {
            Turret_target turret = new Turret_target(hardwareMap, 0);
            Limelight limelight = new Limelight(hardwareMap, 0);
            
            // Aguarda os primeiros 7.5 segundos do autônomo sem travar a CPU
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while (elapsedTime.seconds() < 7.5 && opModeIsActive()){
                idle();
            }

            // Executa a transição para -41° com a rampa afim (goToTarget)
            turret.goToTargetAngle(-39.5, 0.7, 0.35);

            // Mantém -41° via PID até a posição shoot2 ser atingida
            while (opModeIsActive() && elapsedTime.time() < 22){
                turret.updatePID(-39.5);
                sleep(10);
            }
            turret.goToTargetAngle(90, 0.7, 0.35);
            while (opModeIsActive() && elapsedTime.time() <= 27.5){
                turret.updatePID(83);
                sleep(10);
            }
            turret.goToTargetAngle(0, 0.7, 0.35);
            while (opModeIsActive()){
                turret.updatePID(0);
                sleep(10);
            }

            // APÓS O MÉTODO GO TO TARGET E APÓS CHEGAR NA POSIÇÃO SHOOT2:
            // Rastreia e mira com Limelight (idêntico ao TeleOp)

        }
    });


    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-64, 12, Math.toRadians(0));
        Pose2d shotpose = new Pose2d(56, 48, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        VelConstraint gardenVel = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 35;
            }
        };
        VelConstraint maxVel = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 110;
            }
        };
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-40, 60);
        AngularVelConstraint angularVelConstraint = new AngularVelConstraint(Math.toRadians(180));

        TrajectoryActionBuilder estacionar = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-42, 45), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(36, 60), Math.toRadians(0));

        TrajectoryActionBuilder gardens = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-64, 50), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(-65.5, 61), gardenVel)
                .strafeTo(new Vector2d(-65.5, 56), gardenVel)
                .strafeTo(new Vector2d(-64, 62), maxVel)
                .waitSeconds(0.3);


        TrajectoryActionBuilder flor = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(0, 48), maxVel)
                .splineToSplineHeading(new Pose2d(new Vector2d(-19, 44), Math.toRadians(90)), Math.toRadians(0), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(-19, 61), maxVel)
                .strafeTo(new Vector2d(-19, 63), maxVel)
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-19, 53), maxVel, baseAccelConstraint)
                .waitSeconds(0.8);



        TrajectoryActionBuilder second_shot = drive.actionBuilder(new Pose2d(-19, 53, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(0, 50), Math.toRadians(180), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(56, 48), maxVel, baseAccelConstraint);

        TrajectoryActionBuilder second_flor = drive.actionBuilder(new Pose2d(56, 38, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(28, 20), Math.toRadians(0), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(56, 20), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(58, 20), maxVel, baseAccelConstraint)
                .strafeTo(new Vector2d(49, 20))
                .waitSeconds(1);

        TrajectoryActionBuilder third_shot = drive.actionBuilder(new Pose2d(49, 20, Math.toRadians(0)))
                .turnTo(Math.toRadians(270));

        TrajectoryActionBuilder parking = drive.actionBuilder(new Pose2d(new Vector2d(50,18), Math.toRadians(270)))
                .strafeTo(new Vector2d(40, 66), maxVel, baseAccelConstraint);





        Action flower = flor.build();
        Action garden = gardens.build();
        Action shoot2 = second_shot.build();
        Action shoot3 = third_shot.build();
        Action flower2 = second_flor.build();
        Action park = parking.build();


        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        shooter.servo_rampa.setPosition(1);

        waitForStart();
        turret.start();

        Actions.runBlocking(
                new SequentialAction(
                        shooter.prepareshot(2525),
                        shooter.shoot(),
                        intake.intake_on(),
                        intake.rampaBaixo(),
                        flower,
                        shooter.prepareshot(2503),
                        shoot2,
                        intake.intake_off(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                enableLimelightAim = true; // Ativa a miras por Limelight após a posição shoot2
                                return false;
                            }
                        },
                        shooter.shoot(),
                        intake.intake_on(),
                        shooter.prepareshot(2400),
                        flower2,
                        intake.intake_off(),
                        shoot3,
                        shooter.shoot(),
                        park




                        /*shoot2,
                        shooter.shoot(),
                        flower,
                        shoot3,
                        shooter.shoot(),
                        flower2,
                        shoot4,
                        shooter.shoot()*/
                )
        );


    }

    class Intake {
        public DcMotorEx motor_intake, motor_transfer;
        private double TRANSFER_P = 80;
        private double TRANSFER_I = 0.0;
        private double TRANSFER_D = 0.0;
        private double TRANSFER_F = 13;
        private Servo servoflor;

        public Intake (HardwareMap hardwareMap) {
            motor_intake = hardwareMap.get(DcMotorEx.class, "intake");
            motor_transfer = hardwareMap.get(DcMotorEx.class, "transfer");
            motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_transfer.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_transfer.setVelocityPIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);
            servoflor = hardwareMap.get(Servo.class, "servoflor1");
            servoflor.setPosition(1);


        }
        Action intake_on() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    motor_intake.setPower(1);
                    motor_transfer.setVelocity(1900);
                    initialized = true;
                    return false;
                }
            };
        }
        Action rampaBaixo() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servoflor.setPosition(0);
                    return false;
                }
            };
        }

        Action intake_off() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor_intake.setPower(0);
                    motor_transfer.setVelocity(0);
                    initialized = true;
                    return false;
                }
            };
        }

    }
    class Shooter {
        private DcMotorEx motor_shooter, motor_passador_, motor_transfer, intake;
        private Passador passador;
        private CustomShooterV2 shooter;
        private Servo servo_rampa;

        // Transfer PIDF
        public double SHOOTER_P = 0.0009;
        public double SHOOTER_I = 0.00001;
        public double SHOOTER_D = 0.0001;
        public double SHOOTER_F = 0.00019;
        public double SHOOTER_RECOVERY = 0.08;
        public double SHOOTER_MAX_BOOST = 0.25;

        /*public double SHOOTER_P = 120;
        public double SHOOTER_I = 0;
        public double SHOOTER_D = 0.01;
        public double SHOOTER_F = 13.4291;*/


        public double TRANSFER_P = 80;
        public double TRANSFER_I = 0.0;
        public double TRANSFER_D = 0.0;
        public double TRANSFER_F = 13;

        public  double PASSADOR_P = 0.008;
        public  double PASSADOR_I = 0.0;
        public  double PASSADOR_D = 0;
        public  double PASSADOR_F = 0.0001;
        public  double PASSADOR_RECOVERY = 0.1;
        public  double PASSADOR_MAX_BOOST = 0.4;


        public Shooter(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            motor_passador_ = hardwareMap.get(DcMotorEx.class, "passador");
            motor_transfer = hardwareMap.get(DcMotorEx.class, "transfer");
            servo_rampa = hardwareMap.get(Servo.class, "servorampa");
            servo_rampa.setPosition(1);
            passador = new Passador(motor_passador_);
            shooter = new CustomShooterV2(motor_shooter);
            motor_transfer.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_passador_.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_passador_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_passador_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_passador_.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_transfer.setVelocityPIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);
            shooter.setPIDF(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F, SHOOTER_RECOVERY, SHOOTER_MAX_BOOST);
            passador.setPIDF(PASSADOR_P, PASSADOR_I, PASSADOR_D, PASSADOR_F, PASSADOR_RECOVERY, PASSADOR_MAX_BOOST);
        }


        public Action prepareshot(double rpm) {
            return new Action() {

                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    shooter.setTargetRPM(rpm);
                    passador.setTargetRPM(5000);
                    initialized = true;
                    return false;
                }
            };
        }

        public Action shoot() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    while (!shooter.isReady() && !passador.isReady() && opModeIsActive()){
                        passador.update();
                        shooter.update();
                    }
                    servo_rampa.setPosition(0.57);
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.time() < 0.8){
                        passador.update();
                        shooter.update();
                    }
                    timer.reset();
                    while (opModeIsActive() && timer.time() <= 0.5){
                        passador.update();
                        shooter.update();
                        intake.setPower(1);
                        motor_transfer.setVelocity(convertRPMtoTicks(900));
                    }
                    while (opModeIsActive() && timer.time() <= 2.5 && timer.time() > 0.5 ){
                        passador.update();
                        shooter.update();
                        motor_transfer.setVelocity(convertRPMtoTicks(1700));
                    }
                    while (opModeIsActive() && timer.time() <= 3 && timer.time() > 2.5){
                        passador.update();
                        shooter.update();
                        motor_transfer.setVelocity(convertRPMtoTicks(3200));
                    }
                    motor_transfer.setVelocity(0);
                    servo_rampa.setPosition(1);
                    intake.setPower(0);
                    return false;
                }

            };
        }
        public Action shoot1() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    while (!shooter.isReady() && !passador.isReady() && opModeIsActive()){
                        passador.update();
                        shooter.update();
                    }
                    servo_rampa.setPosition(0.57);
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.time() < 0.8){
                        passador.update();
                        shooter.update();
                    }
                    timer.reset();
                    while (opModeIsActive() && timer.time() < 0.5){
                        passador.update();
                        shooter.update();
                        intake.setPower(1);
                        motor_transfer.setVelocity(convertRPMtoTicks(900));
                    }
                    while (opModeIsActive() && timer.time() < 3 && timer.time() > 0.5 ){
                        passador.update();
                        shooter.update();
                        motor_transfer.setVelocity(convertRPMtoTicks(1400));
                    }
                    motor_transfer.setVelocity(0);
                    servo_rampa.setPosition(1);
                    intake.setPower(0);
                    return false;
                }

            };
        }
        public double convertRPMtoTicks(double rpm) {
            return (rpm * 28.0) / 60.0;
        }


    }
    class Turret_target{
        CRServo torreta, torreta1;
        DcMotorEx encoder;
        double start_angle;
        // Engrenagem da torreta (100 dentes) / Engrenagem do encoder (10 dentes) = Relação de 10:1
        // 8192 ticks/rev * 10 voltas = 81920 ticks por volta completa da torreta (360°) -> 227.555 ticks/grau
        double ticksPerDegree = (8192.0 * 10.0) / 360.0;
        double initialTicks = 0;
        private double encoderDirection = -1.0; // -1.0 devido à inversão de sentido entre engrenagens externas (100d x 10d)
                
        // Ganhos do PID de Posição da Torreta (iguais ao TeleOp / Turret.java)
        public double kp = 0.02;
        public double ki = 0.0;
        public double kd = 0.001;
        public double kf = 0.02;
        private double lastError = 0;
        private double integral = 0;

        public Turret_target(HardwareMap hardwareMap, double start_angle){
            this.start_angle = start_angle;
            torreta = hardwareMap.get(CRServo.class, "torreta");
            torreta1 = hardwareMap.get(CRServo.class, "torreta1");
            torreta.setDirection(DcMotorSimple.Direction.FORWARD);
            torreta1.setDirection(DcMotorSimple.Direction.REVERSE);
            encoder = hardwareMap.get(DcMotorEx.class, "intake");
            if (encoder != null) {
                encoder.setDirection(DcMotorSimple.Direction.REVERSE);
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                initialTicks = encoder.getCurrentPosition();
            }
        }

        /**
         * Retorna o ângulo absoluto atual (0° = Reta pro robô, Positivo = Direita, Negativo = Esquerda).
         */
        public double getCurrentAngle() {
            if (encoder == null) return start_angle;
            return start_angle + (encoderDirection * (encoder.getCurrentPosition() - initialTicks) / ticksPerDegree);
        }

        /**
         * Permite inverter manualmente o sinal do encoder caso a torreta gire no sentido oposto.
         */
        public void setInvertEncoder(boolean invert) {
            this.encoderDirection = invert ? -1.0 : 1.0;
        }

        /**
         * Move a torreta para o ângulo absoluto alvo com rampa de desaceleração afim
         * e finaliza com ajuste fino e manutenção por PID de posição (igual ao TeleOp).
         */
        public void goToTargetAngle(double targetAngle, double maxPower, double minPower) {
            double currentAngle = getCurrentAngle();
            double delta = targetAngle - currentAngle;

            double initialDistance = Math.abs(delta);
            double rampDistance = Math.min(5.0, initialDistance);

            ElapsedTime timeout = new ElapsedTime();
            timeout.reset();

            // 1. FASE DE APROXIMAÇÃO: Rampa de desaceleração afim até 2.0° do alvo
            while (opModeIsActive() && timeout.seconds() < 4.0) {
                currentAngle = getCurrentAngle();
                delta = targetAngle - currentAngle;

                double currentDistance = Math.abs(delta);
                // Transita para a fase de PID quando estiver a 2.0° ou menos do alvo
                if (currentDistance <= 2.0) break;

                // Proteção automática: se a distância estiver aumentando em vez de diminuir, inverte a direção do encoder
                if (timeout.seconds() > 0.3 && currentDistance > initialDistance + 10.0) {
                    encoderDirection = -encoderDirection;
                    initialDistance = currentDistance;
                }

                double power;
                if (currentDistance > 5.0) {
                    power = maxPower;
                } else {
                    // Função afim de desaceleração entre maxPower e minPower
                    power = rampDistance > 0 ? minPower + ((maxPower - minPower) / rampDistance) * currentDistance : minPower;
                }

                power = Math.min(maxPower, Math.max(minPower, power));

                if (delta > 0) {
                    // Positivo: virar para a direita
                    torreta.setPower(power);
                    torreta1.setPower(power);
                } else {
                    // Negativo: virar para a esquerda
                    torreta.setPower(-power);
                    torreta1.setPower(-power);
                }

                sleep(10);
            }

            // 2. FASE DE ALINHAMENTO FINO E MANUTENÇÃO POR PID DE POSIÇÃO (Lógica do TeleOp / Turret.java)
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            lastError = 0;
            integral = 0;

            while (opModeIsActive() && holdTimer.milliseconds() < 300) {
                currentAngle = getCurrentAngle();
                double error = targetAngle - currentAngle;

                // Tolerância de precisão fina (<= 0.5°)
                if (Math.abs(error) <= 0.5) break;

                integral += error * 0.01;
                integral = Math.max(-0.2, Math.min(0.2, integral)); // Anti-windup

                double derivative = (error - lastError) / 0.01;
                lastError = error;

                double p = kp * error;
                double i = ki * integral;
                double d = kd * derivative;
                double f = kf * Math.signum(error);

                double outputPower = p + i + d + f;
                outputPower = Math.min(maxPower, Math.max(-maxPower, outputPower));

                torreta.setPower(outputPower);
                torreta1.setPower(outputPower);

                sleep(10);
            }

            torreta.setPower(0);
            torreta1.setPower(0);
        }

        /**
         * Realiza a calibração de Feedforward Estático (kF / Stiction) da torreta.
         * Instancia os servos e encoder idênticos ao TeleOp e incrementa a potência
         * gradualmente até detectar o menor movimento no encoder.
         *
         * @return O valor de kF estático mínimo encontrado
         */
        public double calibrateStaticFeedforward() {
            if (encoder == null) return 0.02;

            int startTicks = encoder.getCurrentPosition();
            double testPower = 0.0;
            double step = 0.002;

            while (opModeIsActive() && testPower <= 0.40) {
                testPower += step;
                torreta.setPower(testPower);
                torreta1.setPower(testPower);

                sleep(40);

                int deltaTicks = Math.abs(encoder.getCurrentPosition() - startTicks);
                // Quando o encoder registra variação (>= 10 ticks), determinou a potência mínima de movimento
                if (deltaTicks >= 10) {
                    torreta.setPower(0);
                    torreta1.setPower(0);
                    this.kf = testPower; // Define kf com o valor de atrito medido
                    return testPower;
                }
            }

            torreta.setPower(0);
            torreta1.setPower(0);
            return testPower;
        }

        /**
         * Mantém continuamente a torreta alinhada no ângulo alvo utilizando PID de posição (igual ao TeleOp).
         */
        public void updatePID(double targetAngle) {
            double currentAngle = getCurrentAngle();
            double error = targetAngle - currentAngle;

            if (Math.abs(error) <= 0.5) {
                torreta.setPower(0);
                torreta1.setPower(0);
                return;
            }

            integral += error * 0.015;
            integral = Math.max(-0.2, Math.min(0.2, integral)); // Anti-windup

            double derivative = (error - lastError) / 0.015;
            lastError = error;

            double p = kp * error;
            double i = ki * integral;
            double d = kd * derivative;
            double f = kf * Math.signum(error);

            double outputPower = Math.min(0.5, Math.max(-0.5, p + i + d + f));
            torreta.setPower(outputPower);
            torreta1.setPower(outputPower);
        }

        public Action goToTargetAngleAction(double targetAngle, double maxPower, double minPower) {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                    }
                    if (initialized) {
                        goToTargetAngle(targetAngle, maxPower, minPower);
                        return true;
                    }
                    return false;
                }
            };
        }
    }
}
