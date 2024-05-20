using UnityEngine;

public class PID3D
{
    private float kp;
    private float ki;
    private float kd;

    private Vector3 integral;
    private Vector3 previousError;

    public PID3D(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        integral = Vector3.zero;
        previousError = Vector3.zero;
    }

    public Vector3 Update(Vector3 target, Vector3 current, float deltaTime)
    {
        Vector3 error = target - current;

        integral += error * deltaTime;
        Vector3 derivative = (error - previousError) / deltaTime;

        Vector3 output = kp * error + ki * integral + kd * derivative;

        previousError = error;

        return output;
    }

    public void Reset()
    {
        integral = Vector3.zero;
        previousError = Vector3.zero;
    }
}
