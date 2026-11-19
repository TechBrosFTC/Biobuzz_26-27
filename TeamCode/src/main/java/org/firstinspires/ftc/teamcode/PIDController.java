package org.firstinspires.ftc.teamcode;

/**
 * This is the class for the PID controller. This class is used to calculate the output of the PID controller on mechnisms
 * that need to keep track of the error, like the mecanum drive base or arms.
 * @author Gabriel Borges
 */
public  class PIDController {
    double kp;
    double kd;
    double ki;
    double prevError;
    double error;
    double integral;
    double proportional;
    double derivative;
    double output;


    /** As constantes devem ser testadas pelo FTC Dashboard após a definição do setPoint para assegurar a correção
     * @param kp Contante proporcional do controlador PID
     *  @param kd Contante derivativa do controlador PID
     *  @param ki Contante integral do controlador PID
     * **/
    public PIDController(double kp, double kd, double ki){
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
    }
    public double calculate(double error, double target){//*this method returns the output of the PID controller
        this.error = target - error;
        proportional = this.kp * error;
        derivative = this.kd * (error - prevError);
        integral += this.ki * error;
        output = proportional + derivative + integral;
        prevError = error;
        return output;
    }

}