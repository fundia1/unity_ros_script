using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;
using System.Collections.Generic;
public class Test_agent : Agent
{
    public Transform target;
    private Rigidbody rBody;

    public GameObject frontLeftWheel;
    public GameObject frontRightWheel;
    public GameObject rearLeftWheel;
    public GameObject rearRightWheel;




    private WheelCollider frontLeftWheelCollider;
    private WheelCollider frontRightWheelCollider;
    private WheelCollider rearLeftWheelCollider;
    private WheelCollider rearRightWheelCollider;

    public float maxLinearSpeed = 25f;
    public float maxRotation = 28f;
    public float brakeForce = 5000f;

    
    private float wheelRadius;

    private float currentSpeed = 0f;
    private float currentSteer = 0f;
    

    public float distanceToTarget; 


    public float rewardDistance = 1.0f; // 목표와 일정 거리 이내일 때 보상을 주는 거리

    void Start()
    {
        rBody = GetComponent<Rigidbody>();

        frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
        frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
        rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
        rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

        wheelRadius = frontLeftWheelCollider.radius;
    }

    

    private void FixedUpdate()
    {
        distanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);

        UpdateVisualWheels();
    }



    public override void OnEpisodeBegin()
    {
        
        // if (this.transform.localPosition.y < 0.29f)
        // {
        //     this.rBody.angularVelocity = Vector3.zero;
        //     this.rBody.linearVelocity = Vector3.zero; // linearVelocity → velocity 수정
        //     this.transform.localPosition = new Vector3(0f  , 0.3f, 0f);
        //     float randomYRotation = Random.Range(0f, 360f);  // 0도에서 360도 사이의 랜덤 값 생성
        //     this.transform.localRotation = Quaternion.Euler(0f, randomYRotation, 0f);

            
        // }
        
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.linearVelocity = Vector3.zero; // linearVelocity → velocity 수정
        this.transform.localPosition = new Vector3(0f  , 0.3f, 0f);
        float randomYRotation = Random.Range(0f, 360f);  // 0도에서 360도 사이의 랜덤 값 생성
        this.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

        target.localPosition = new Vector3(Random.Range(-25f, 25f), 0.3f, Random.Range(1f, 25f));

        // Debug.Log("시작 ");

        
        
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 목표물 및 차량 위치
        sensor.AddObservation(target.localPosition.x);
        sensor.AddObservation(target.localPosition.z);

        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);

       
        sensor.AddObservation(transform.forward.z);
        
        
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        
        int steer = actionBuffers.DiscreteActions[0]; // 조향 조절값 0~10
        
        
        
        float targetSpeed = 10 / 3.6f;
        float targetSteer = (steer - 5) * (28f / 5f); // -28 ~ 28


        Debug.Log($"조향각: {steer} 찐조향각: {targetSteer}");
        // 속도 및 조향 적용
        RobotInput(targetSpeed, targetSteer);

        

        // 디버깅 정보 출력
        float actualSpeed = rBody.linearVelocity.magnitude *3.6f;
        // Debug.Log($"[디버그], 속도: {actualSpeed}, 조향각: {targetSteer}");
        Debug.Log($"[디버그], 옆: {transform.forward.x}, 앞: {transform.forward.z}");

        

        if (distanceToTarget < rewardDistance) // front가 target과 가까워졌을 때
        {
            SetReward(100.0f);
            Debug.Log("목표에 근접! 보상 +100");
            EndEpisode();
        }

        // 차량이 떨어지면 에피소드 종료 (Y 좌표가 특정 값 이하로 떨어지면)
        if (this.transform.localPosition.y < 0.29f)
        {
            EndEpisode(); // 에피소드 종료
        }

        
    }



    

    private void RobotInput(float speed, float rot)
    {
        float maxTorque = Mathf.Abs(speed) * 8f; 
        currentSpeed = rBody.linearVelocity.magnitude * Mathf.Sign(rBody.linearVelocity.z);
        // Debug.Log($"Robot Input - Speed: {currentSpeed * 3.6f} km/h");
        float speedError = Mathf.Abs(speed - currentSpeed);
        float torqueFactor = 1.0f - Mathf.Log10(speedError + 1.0f); // 1 - log(n) 형태의 감속 곡선
        
        // 토크가 0보다 작아지는 것을 방지
        torqueFactor = Mathf.Clamp(torqueFactor, 0.1f, 1.0f);
        
        float appliedTorque =Mathf.Clamp(Mathf.Abs(speed), 0.0f, 1.0f) * maxTorque * torqueFactor * Mathf.Sign(speed - currentSpeed);
        
        
        ApplyMotorTorque(frontLeftWheelCollider, appliedTorque);
        ApplyMotorTorque(frontRightWheelCollider, appliedTorque);
        ApplyMotorTorque(rearLeftWheelCollider, appliedTorque);
        ApplyMotorTorque(rearRightWheelCollider, appliedTorque);

        frontLeftWheelCollider.steerAngle = rot;
        frontRightWheelCollider.steerAngle = rot;

        // Debug.Log($"Robot Input - Speed: {speed}, Rotation: {rot}");
    }

    private void ApplyMotorTorque(WheelCollider wheelCollider, float speed)
    {
        wheelCollider.motorTorque = speed;
    }

    private void ApplyBrake(float brakeTorque)
    {
        frontLeftWheelCollider.brakeTorque = brakeTorque;
        frontRightWheelCollider.brakeTorque = brakeTorque;
        rearLeftWheelCollider.brakeTorque = brakeTorque;
        rearRightWheelCollider.brakeTorque = brakeTorque;
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
            float wheelAngularSpeed = (wheelCollider.rpm * wheelRadius * 2f * Mathf.PI) / 60f;
            wheel.transform.Rotate(Vector3.right, wheelAngularSpeed * Time.fixedDeltaTime, Space.Self);

            if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
            {
                wheel.transform.localRotation = Quaternion.Euler(0f, wheelCollider.steerAngle, 0f);
            }
        }
    }
 
}