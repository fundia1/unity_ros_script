using UnityEngine;
using UnityEngine.InputSystem;

namespace RosSharp.Control
{
    public class KeyboardMove : MonoBehaviour
    {
        public GameObject frontLeftWheel;
        public GameObject frontRightWheel;
        public GameObject rearLeftWheel;
        public GameObject rearRightWheel;

        private WheelCollider frontLeftWheelCollider;
        private WheelCollider frontRightWheelCollider;
        private WheelCollider rearLeftWheelCollider;
        private WheelCollider rearRightWheelCollider;

        public float maxLinearSpeed = 25f; // km/h
        public float maxRotationalSpeed = 1f; // rad/s
        public float brakeForce = 5000f; // 브레이크 토크

        private float inputSpeed = 0f;
        private float inputRotationSpeed = 0f;
        private bool isBraking = false;

        private float wheelRadius = 0.3f; // 바퀴 반지름 (m)
        private float wheelCircumference;

        private Keyboard keyboard;

        private Rigidbody rb;

        public float currentSpeed = 0f;    // 현재 속도 (km/h)
        public float speedSmoothingFactor = 10f; // 속도 조정 비율 (0에서 1 사이)
        private float torqueFactor = 1f;   // 토크 계수 (최대 1로 설정)

        void Start()
        {
            keyboard = Keyboard.current;


            rb = GetComponent<Rigidbody>();

            // WheelCollider를 바퀴에 연결
            frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
            frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
            rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
            rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

            if (frontLeftWheelCollider == null || frontRightWheelCollider == null || rearLeftWheelCollider == null || rearRightWheelCollider == null)
            {
                Debug.LogError("Missing WheelCollider on one or more wheels!");
            }

            
           
        }

        void FixedUpdate()
        {
            // 키보드 입력 처리
            KeyBoardUpdate();
            // 바퀴 시각적 업데이트
            UpdateVisualWheels();

            LogVehicleSpeed();
        }


        private void LogVehicleSpeed()
        {
            if (rb != null)
            {
                float speed = rb.linearVelocity.magnitude * 3.6f; // m/s 단위 속도
                Debug.Log($"현재 속도: {speed} km/h");
            }
        }
        

        private WheelFrictionCurve GetFrictionCurve(float extremumSlip, float extremumValue, float asymptoteSlip, float asymptoteValue)
        {
            WheelFrictionCurve frictionCurve = new WheelFrictionCurve
            {
                extremumSlip = extremumSlip,
                extremumValue = extremumValue,
                asymptoteSlip = asymptoteSlip,
                asymptoteValue = asymptoteValue,
                stiffness = 1.5f // 마찰력 강도 증가
            };
            return frictionCurve;
        }

        private void KeyBoardUpdate()
        {
            if (keyboard == null)
            {
                Debug.LogError("Keyboard not found.");
                return;
            }

            // 스페이스바 입력 확인 (브레이크)
            if (keyboard.spaceKey.isPressed)
            {
                isBraking = true;
                ApplyBrake(brakeForce);
                inputSpeed = 0;
                inputRotationSpeed = 0;
                return;
            }
            else
            {
                isBraking = false;
                ApplyBrake(0); // 브레이크 해제
            }

            // 전진/후진 속도 설정
            if (keyboard.wKey.isPressed)
            {
                inputSpeed = maxLinearSpeed; // 전진
            }
            else if (keyboard.sKey.isPressed)
            {
                inputSpeed = -maxLinearSpeed; // 후진
            }
            else
            {
                inputSpeed = 0;
            }

            // 회전 속도 설정
            if (keyboard.aKey.isPressed)
            {
                inputRotationSpeed = -maxRotationalSpeed; // 좌회전
            }
            else if (keyboard.dKey.isPressed)
            {
                inputRotationSpeed = maxRotationalSpeed; // 우회전
            }
            else
            {
                inputRotationSpeed = 0;
            }

            // 로봇 입력 처리
            RobotInput(inputSpeed, inputRotationSpeed);
        }

       private void RobotInput(float speed, float rotSpeed)
{
    // 최대 회전 속도 및 토크 값
    const float MAX_RPM = 3000f;
    const float PEAK_RPM = 5000f;
    const float MAX_TORQUE = 9.55f;
    const float PEAK_TORQUE = 40f;

    // 차량의 전방 벡터
    Vector3 forward = transform.forward;
    // 현재 속도 벡터
    Vector3 velocity = rb.velocity;

    // 전방 벡터와 속도 벡터 간의 각도 계산
    float angle = Vector3.Angle(forward, velocity);

    // 속도의 부호 결정 (전방 벡터와 속도 벡터가 같은 방향이면 양수, 반대 방향이면 음수)
    float sign = (Vector3.Dot(forward, velocity) >= 0) ? 1.0f : -1.0f;

    // 현재 속도 계산
    currentSpeed = velocity.magnitude * sign;

    // 속도 오차 계산
    float speedError = speed - currentSpeed;

    // PID 제어기 변수 (적절한 값으로 조정 필요)
    float Kp = 2.5f; // 비례 계수 증가
    float Ki = 0.1f; // 적분 계수 증가
    float Kd = 0.01f; // 미분 계수 증가

    // PID 제어기 계산
    float proportional = Kp * speedError;
    float integral = Ki * speedError * Time.deltaTime;
    float derivative = Kd * (speedError - previousSpeedError) / Time.deltaTime;
    previousSpeedError = speedError;

    float pidOutput = proportional + integral + derivative;

    // 최대 토크 계산
    float maxTorque = Mathf.Lerp(MAX_TORQUE, PEAK_TORQUE, Mathf.Abs(speed) / PEAK_RPM) * 20;

    // 적용할 토크 계산
    float appliedTorque = Mathf.Clamp(pidOutput * 20, -maxTorque, maxTorque);

    // 목표 속도가 0일 때 토크를 0으로 설정
    if (Mathf.Approximately(speed, 0f))
    {
        appliedTorque = 0f;
    }

    // 뒤 바퀴 회전 속도 설정
    ApplyMotorTorque(rearLeftWheelCollider, appliedTorque);
    ApplyMotorTorque(rearRightWheelCollider, appliedTorque);

    // 조향 각도 설정 (최대 28도 제한)
    float steerAngle = Mathf.Clamp(rotSpeed * Mathf.Rad2Deg, -28f, 28f);
    frontLeftWheelCollider.steerAngle = steerAngle;
    frontRightWheelCollider.steerAngle = steerAngle;
}

// PID 제어기 변수 초기화
private float previousSpeedError = 0f;
        private void ApplyMotorTorque(WheelCollider wheelCollider, float speed)
        {
            if (wheelCollider != null && !isBraking)
            {
                // speed를 motorTorque로 적용
                wheelCollider.motorTorque = speed;
            }
            else
            {
                // 브레이크 시 motorTorque를 0으로 설정
                wheelCollider.motorTorque = 0f;
            }
        }

        private void ApplyBrake(float brakeTorque)
        {
            if (frontLeftWheelCollider != null) frontLeftWheelCollider.brakeTorque = brakeTorque;
            if (frontRightWheelCollider != null) frontRightWheelCollider.brakeTorque = brakeTorque;
            if (rearLeftWheelCollider != null) rearLeftWheelCollider.brakeTorque = brakeTorque;
            if (rearRightWheelCollider != null) rearRightWheelCollider.brakeTorque = brakeTorque;

            Debug.Log($"Braking with force: {brakeTorque}");
        }

        private void UpdateVisualWheels()
        {
            UpdateWheelRotation(frontLeftWheel, frontLeftWheelCollider);
            UpdateWheelRotation(frontRightWheel, frontRightWheelCollider);
            UpdateWheelRotation(rearLeftWheel, rearLeftWheelCollider);
            UpdateWheelRotation(rearRightWheel, rearRightWheelCollider);
        }

        private void UpdateWheelRotation(GameObject wheel, WheelCollider wheelCollider)
        {
            if (wheelCollider != null && wheel != null)
            {
                // WheelCollider의 RPM을 사용하여 실제 바퀴의 회전 각도 계산
                float wheelAngularSpeed = (wheelCollider.rpm * wheelRadius * 2f * Mathf.PI) / 60f; // m/s 단위로 회전 속도 계산
                wheel.transform.Rotate(Vector3.right, wheelAngularSpeed * Time.fixedDeltaTime, Space.Self);

                // 전륜 바퀴에 steerAngle 적용
                if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
                {
                    wheel.transform.localRotation = Quaternion.Euler(0f, wheelCollider.steerAngle, 0f);
                }
            }
        }
    }
}
