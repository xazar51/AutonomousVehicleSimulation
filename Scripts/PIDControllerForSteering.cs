using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDControllerForSteering : MonoBehaviour
{
    float error_old = 0f;
    float error_sum = 0f;

    //PID parameters
    public float K_U = 0f;
    public float T_U = 0f;
    //public float tau_D = 0f;

    public float GetSteerAngleFromPIDController(float error)
    {
        float output = 0f;

        //P term
        output = 0.6f * K_U * error;

        //I term
        error_sum += Time.fixedDeltaTime * error;

        float averageAmount = 20f;

        error_sum = error_sum + ((error - error_sum) / averageAmount);


        output += (1.2f * K_U / T_U) * error_sum;

        //D
        float d_error_dt = (error - error_old) / Time.fixedDeltaTime;

        output += (3 * K_U * T_U / 40) * d_error_dt;

        error_old = error;

        return output;
    }
}
