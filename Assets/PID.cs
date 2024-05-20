[System.Serializable]
public class PID {
	public float kp, ki, kd;
		
	float integral;
	float et_1;
	
	
	public PID(float kp, float ki, float kd) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
	}
	
	public float Update(float r, float y, float dt) {
		float et = r - y;
		integral += et * dt;
		et_1 = et;
		return kp * et + ki * integral + kd * (et - et_1) / dt;
	}
}
