
package org.firstinspires.ftc.teamcode;

public class PID {

    private double kp;
    private double kd;

    private double previousRPM = 0.0;
    private boolean firstUpdate = true;

    public PID(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
    }

    public double update(double error, double currentRPM, double deltaTime) {

        if (deltaTime <= 0) {
            return 0.0;
        }

        // Proporcional
        double pComponent = kp * error;

        // Derivativo sobre a velocidade medida
        // Evita o pico derivativo quando o alvo muda
        double derivative = 0.0;

        if (!firstUpdate) {
            derivative = (currentRPM - previousRPM) / deltaTime;
        }

        double dComponent = -kd * derivative;

        previousRPM = currentRPM;
        firstUpdate = false;

        return pComponent + dComponent;
    }
   public void setCoefficients(double kp, double kd){
        this.kp = kp;
        this.kd = kd;
    }

    public void reset() {
        previousRPM = 0.0;
        firstUpdate = true;
    }
}
