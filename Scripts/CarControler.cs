using HealthbarGames;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarControler : MonoBehaviour
{
    #region public members

    public Rigidbody car;

    //Vehicle data
    public float maxSteerAngle;
    public float motorTorque;
    public float brakeTorque;

    // The path to follow
    public GameObject path;

    // distance between the Node and the center of the vehicle
    // In other words, it indicates difference how early the car starts to steering, when it reaches a node where steering is needed.
    // If this difference is big, then the car will start to steer too early
    public float steeringReactionOffset;

    [Header("Sensors")]
    public float SensorLength = 5f;
    public Vector3 FrontSensorPosition = new Vector3(1.8f, 0.5f, 2);
    public float FrontSideSensorsOffset = 0.47f;
    public float FrontSideSensorAngle = 30f;

    // Physical components of the wheels
    [Header("Wheel Tranforms")]
    public Transform FrontDriverT;
    public Transform FrontPassangerT;
    public Transform RearDriverT;
    public Transform RearPassangerT;

    // Wheel Collider component of each wheel
    [Header("Wheel Colliders")]
    public WheelCollider FrontDriverW;
    public WheelCollider FrontPassangerW;
    public WheelCollider RearDriverW;
    public WheelCollider RearPassangerW;

    #endregion

    #region private members
    // Nodes that the path is formed of
    private List<Transform> m_nodes = new List<Transform>();
    // Each node has a PathNode component, which contains the target speed of the node.
    private PathNode[] m_velocityList;
    private float m_currentTargetVelocity;
    private int m_currentNodeIndex = 0;
    // true, when the vehicle reachs the last node of the path
    private bool m_ReachedToEnd;
    private Vector3 m_steerPosition;
    private Vector3 m_currentNode;
    private Vector3 m_previousNode;
    // The angle that we need to steer
    private float m_steeringAngle = 0f;
    //For controlling the speed of the car
    private float m_throttle = 0f;
    // true, if the vehicle is increasing the speed.
    private bool m_isAccelerating;

    private PIDControllerForSteering m_PIDControllerForSteering;
    private PIDControllerForSpeed m_PIDControllerForSpeed;

    //warner object is placed before the every traffic light, so when the vehicle collide with it,
    //the vehicle starts to set its speed in every frame based on the curret state of the traffic light.
    private TrafficLightWarner m_currentTLwarner;

    public PedestrainPathSeg m_currentPedestrainPathSeg;
    private bool m_fullBrake;

    #endregion

    #region methods

    /// <summary>
    /// This method is called first and just once when the script is enabled.
    /// </summary>
    private void Start()
    {
        Transform[] nodeTransforms = path.GetComponentsInChildren<Transform>();
        foreach (Transform trans in nodeTransforms)
        {
            if (trans != path.transform)
            {
                m_nodes.Add(trans);
            }
        }

        m_velocityList = path.GetComponentsInChildren<PathNode>();

        m_currentNode = m_nodes[m_currentNodeIndex].position;

        m_previousNode = GetPreviousNode();
        m_PIDControllerForSteering = GetComponent<PIDControllerForSteering>();
        m_PIDControllerForSpeed = GetComponent<PIDControllerForSpeed>();
    }

    /// <summary>
    /// Assigns the calculated steering angle value to the corresponding property of the front wheels
    /// </summary>
    private void Steer()
    {
        FrontDriverW.steerAngle = m_steeringAngle;
        FrontPassangerW.steerAngle = m_steeringAngle;
    }

    /// <summary>
    /// Applies motor torque to the rear wheels in order to increase the speeed.
    /// </summary>
    private void Accelerate()
    {

        if ((m_currentPedestrainPathSeg != null && m_currentPedestrainPathSeg.PedestrainPassing) || m_fullBrake)
        {
            Debug.Log("no accelerate");
            return;
        }
        //Also in case of Green and Red-Yellow states of traffic light, the vehicle continue to accelerate
        if (m_isAccelerating)
        {
            //Debug.Log(string.Format("MotorTorque: {0} N/m, velocity: {1} km/h", motorTorque * m_throttle, car.velocity.magnitude * 3.6f));
            FrontDriverW.brakeTorque = 0f;
            FrontPassangerW.brakeTorque = 0f;
            RearDriverW.brakeTorque = 0f;
            RearPassangerW.brakeTorque = 0f;

            RearDriverW.motorTorque = motorTorque * m_throttle;
            RearPassangerW.motorTorque = motorTorque * m_throttle;
        }

    }

    /// <summary>
    /// We need to update the position of physical component (Transform) of wheels with position of the visual ones (Wheel Collider)
    /// This method calls UpdateWheelPosition method for each wheel.
    /// </summary>
    private void UpdateWheelPositions()
    {
        UpdateWheelPosition(FrontDriverW, FrontDriverT);
        UpdateWheelPosition(FrontPassangerW, FrontPassangerT);
        UpdateWheelPosition(RearDriverW, RearDriverT);
        UpdateWheelPosition(RearPassangerW, RearPassangerT);
    }

    /// <summary>
    /// Gets the position of the wheel collider component and assign it to the Transform component
    /// </summary>
    /// <param name="wheelCollider">Visual component</param>
    /// <param name="transform">Physical component</param>
    private void UpdateWheelPosition(WheelCollider wheelCollider, Transform transform)
    {
        Vector3 pos = transform.position;
        Quaternion rot = transform.rotation;

        wheelCollider.GetWorldPose(out pos, out rot);

        transform.position = pos;
        transform.rotation = rot;
    }

    /// <summary>
    /// Applies break torque to the front wheels in order to deaccelerate the vehicle
    /// </summary>
    private void Brake()
    {
  
        if ((m_currentPedestrainPathSeg != null && m_currentPedestrainPathSeg.PedestrainPassing) || m_fullBrake)
		{
            Debug.Log("full brake");
            FrontDriverW.brakeTorque = brakeTorque;
            FrontPassangerW.brakeTorque = brakeTorque;
			
			FrontPassangerW.motorTorque = 0;
            FrontDriverW.motorTorque = 0;
            RearPassangerW.motorTorque = 0;
            RearDriverW.motorTorque = 0;

            return;
			
		}
        if (!m_isAccelerating)
        {
            //Debug.Log(string.Format("BrakeTorque: {0} N/m, velocity: {1} km/h", brakeTorque * m_throttle, car.velocity.magnitude * 3.6f));
            FrontDriverW.brakeTorque = brakeTorque * m_throttle;
            FrontPassangerW.brakeTorque = brakeTorque * m_throttle;

            FrontPassangerW.motorTorque = 0;
            FrontDriverW.motorTorque = 0;
            RearPassangerW.motorTorque = 0;
            RearDriverW.motorTorque = 0;
        }        
    }

    /// <summary>
    /// Update method is called every frame.
    /// </summary>
    private void Update()
    {
        m_steerPosition = transform.position + transform.forward * steeringReactionOffset;

        if(Math.HasPassedNode(m_steerPosition, m_previousNode, m_currentNode))
        {
            m_currentNodeIndex++;
            if(m_currentNodeIndex == m_nodes.Count)
            {
                m_currentNodeIndex = 0;
            }
            m_currentNode = m_nodes[m_currentNodeIndex].position;
            m_previousNode = GetPreviousNode();
        }
    }

    private Vector3 GetPreviousNode()
    {
        m_previousNode = Vector3.zero;

        if (m_currentNodeIndex - 1 < 0)
        {
            m_previousNode = m_nodes[m_nodes.Count - 1].position;
        }
        else
        {
            m_previousNode = m_nodes[m_currentNodeIndex - 1].position;
        }
        return m_previousNode;
    }

    private void FixedUpdate()
    {
        m_currentTargetVelocity = m_velocityList[m_currentNodeIndex].Velocity;

        CheckSensors();
        CheckTrafficLigth();
        CalculateSteeringAngle();   
        CalculateThrottleValue();
        Steer();
        Accelerate();
        Brake();
        UpdateWheelPositions();
    }

    private void CheckTrafficLigth()
    {
        if (m_currentTLwarner != null)
        {
            //Red or Yellow states of the traffic light, so apply brake
            //Todo:  somehow calculate that whether the car should still go or stop in case of yellow state
            if ((m_currentTLwarner.Fwd_TrafficLight.RedHalo.activeSelf &&
                !m_currentTLwarner.Fwd_TrafficLight.YellowHalo.activeSelf) ||
                (m_currentTLwarner.Fwd_TrafficLight.YellowHalo.activeSelf))
            {
                m_currentTargetVelocity = 0;
            }
        }
    }

    private void CalculateThrottleValue()
    {
        float currentVelocity = car.velocity.magnitude * 3.6f;
        float speedError = m_currentTargetVelocity - currentVelocity;

        Debug.Log(speedError);

        m_throttle = m_PIDControllerForSpeed.GetThrottleFactorFromPIDController(speedError);
        m_throttle = Mathf.Clamp(m_throttle, -1, 1);
        if (m_throttle > 0)
        {
            m_isAccelerating = true;
        }
        else
        {
            m_isAccelerating = false; // So we need to deaccelerate
            m_throttle *= -1;
        }
    }

    private void CalculateSteeringAngle()
    {
        float error = Math.GetError(m_steerPosition, m_previousNode, m_currentNode);

        error *= Math.SteerDirection(transform, m_steerPosition, m_currentNode);

        float steeringAngle = m_PIDControllerForSteering.GetSteerAngleFromPIDController(error);

        steeringAngle = Mathf.Clamp(steeringAngle, -maxSteerAngle, maxSteerAngle);

        float averageAmount = 30f;

        m_steeringAngle = m_steeringAngle + ((steeringAngle - m_steeringAngle) / averageAmount);
    }
	
	private void CheckSensors(){
		RaycastHit hit;
        Vector3 sensorStartPos = transform.position + transform.forward * FrontSensorPosition.x  + transform.up * FrontSensorPosition.y;
        m_fullBrake = false;

		//two second rule
		float distanceToKeep = car.velocity.magnitude * 2;
        distanceToKeep = Mathf.Clamp(distanceToKeep, 1, 20);
		//Debug.Log(string.Format("distance: {0}", distanceToKeep));
		
        //Front Middle Sensor
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, distanceToKeep))
        {
            Debug.DrawLine(sensorStartPos, hit.point);
            if (hit.rigidbody != null)
            {
                m_currentTargetVelocity = hit.rigidbody.velocity.magnitude * 3.6f;
                if(hit.distance <= 1)
                {
                    m_fullBrake = true;
                }
            }
        }


    }

    void OnTriggerEnter (Collider collider)
    {
        if(collider.tag == "light_warner")
        {
            m_currentTLwarner = collider.GetComponent<TrafficLightWarner>();

        }
        else if(collider.tag == "light_position")
        {
            m_currentTLwarner = null;
        }
		else if(collider.tag == "pedestrain_warner"){
			m_currentPedestrainPathSeg = collider.GetComponentInParent<PedestrainPathSeg>();
		}
		else if(collider.tag == "pedestrain_path_seg"){
			m_currentPedestrainPathSeg = null;
		}
    }

    #endregion
}
