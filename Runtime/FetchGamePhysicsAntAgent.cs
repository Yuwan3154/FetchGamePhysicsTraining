using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// A trainable agent for the fetch game:
/// <seealso href="https://github.com/JohnsonLabJanelia/FetchGamePhysics/tree/main/FetchArenaProject"/>
/// </summary>
public class FetchGamePhysicsAntAgent : FetchGamePhysicsMuJoCoAgent
{
    
    
    
    /// <summary>
    /// The size of the agent's body mesh.  If the body covers [-S/2, S/2] in a dimension
    /// then the size should be S in that dimension.
    /// </summary>
    public virtual Vector3 BodyScale
    {
        get { return _groundedBodyScale; }
        protected set { _groundedBodyScale = value; }
    }
    private Vector3 _groundedBodyScale = new Vector3(0.1f, 0.1f, 0.1f);

    /// <summary>
    /// The color of the agent's body mesh.
    /// </summary>
    public virtual string BodyColor
    {
        get { return _groundedBodyColor; }
        protected set { _groundedBodyColor = value; }
    }
    private string _groundedBodyColor = "#ff0000";

    /// <summary>
    /// The agent's collider's static friction.
    /// </summary>
    public virtual float StaticFriction
    {
        get { return _groundedStaticFriction; }
        protected set { _groundedStaticFriction = value; }
    }
    private float _groundedStaticFriction = 0.15f;

    /// <summary>
    /// The agent's collider's dynamic friction.
    /// </summary>
    public virtual float DynamicFriction
    {
        get { return _groundedDynamicFriction; }
        protected set { _groundedDynamicFriction = value; }
    }
    private float _groundedDynamicFriction = 0.15f;

    /// <summary>
    /// The direction of the force on the agent for forward movement.
    /// </summary>
    public virtual Vector3 MoveForwardDirection
    {
        get { return transform.forward; }
    }

    /// <summary>
    /// Called after the Setup function for the arena (the class derived from <see cref="EasyMLArena"/>).
    /// If a derived class needs to override this function for further setup, it should call base.Setup.
    /// Creates new objects or updates existing objects.
    /// </summary>
    /// <param name="helper"></param>
    public override void Setup(Janelia.IEasyMLSetupHelper helper)
    {
        // `ColliderSize` is set here, in case the derived `Setup` changes `BodyScale`.
        ColliderSize = BodyScale;

        base.Setup(helper);
        gameObject.name = "AntAgent";

        const string BODY_NAME = "Body";
        Transform bodyTransform = transform.Find(BODY_NAME);
        GameObject body;
        if (bodyTransform == null)
        {
            Instantiate("Assets/Prefabs/Ant.prefab", body.transform);
            body = GameObject.Find("Ant");
            body.name = BODY_NAME;
            body.transform.parent = transform;
        }
        else
        {
            body = bodyTransform.gameObject;
        }
        
        helper.CreateColorMaterial(body, BodyColor);
        body.transform.localScale = new Vector3(BodyScale.x, BodyScale.y, BodyScale.z);

        // Put the overall agent position at the back and bottom of the body and its collider.
        // Doing so helps to keep the agent from "tipping" forward or backward when a force is applied.
        float y = BodyScale.y / 2;
        body.transform.localPosition = new Vector3(0, y, 0);

        Camera childCamera = GetComponentInChildren<Camera>();
        if (childCamera != null)
        {
            childCamera.transform.localPosition = new Vector3(0, 2 * y, y);
        }
    }

    /// <summary>
    /// Called when an action is received from either the player input or the neural network.
    /// It is also common to assign a reward in this method.
    /// </summary>
    /// <param name="actions">Action to take</param>
    ///
    public override void OnActionReceived(ActionBuffers actions)
    {
        if (_frozen)
        {
            return;
        }

        float moveChange = actions.ContinuousActions[0];
        Vector3 dir = (moveChange > 0) ? MoveForwardDirection : transform.forward;
        _agentRigidbody.AddForce(moveChange * moveForce * dir);

        // Changes to yaw (Y rotation) are smoothed over multiple frames.

        float yawChange = actions.ContinuousActions[1];
        Vector3 rotationVector = transform.rotation.eulerAngles;
        _smoothYawChange = Mathf.MoveTowards(_smoothYawChange, yawChange, 2.0f * Time.fixedDeltaTime);
        float yaw = rotationVector.y + _smoothYawChange * Time.fixedDeltaTime * yawSpeed;

        transform.rotation = Quaternion.Euler(0.0f, yaw, 0.0f);
    }

    /// <summary>
    /// When behavior type is set to "Heuristic only" on the agent's behavior parameters,
    /// then this function will be called.  Its return value will be fed into
    /// <see cref="OnActionReceived(ActionBuffers)"/> and the neural network is ignored.
    /// </summary>
    /// <param name="actionsOut">An output action buffer</param>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        /// Log an error to show that Heuristic is not supported.
        Debug.LogError("Heuristic is not supported for this agent.", this);
    }
    
    
    
    
    
    
    public override string BehaviorName { get; protected set; } = "FetchGamePhysics";

    public override int VectorObservationSize { get; protected set; } = 6;

    public override int VectorActionSize 
    {
        get { return _antVectorActionSize; }
        protected set { _antVectorActionSize = value; }
    } 
    private int _antVectorActionSize = 2;

#if false
    // Overriding this virtual property would be an alternative to setting its value in `Setup`, but
    // the latter seems simpler, and `Setup` is needed for a few other purposes anyway.
    public override List<string> ChildSensorForwardDetectableTags { get; protected set; } = new List<string>()
    {
        FetchGamePhysicsTrainingArena.TAG_BOUNDARY,
        FetchGamePhysicsTrainingArena.TAG_RAMP
    };
#endif

    private GameObject _ball;
    private Rigidbody _ballRigidBody;
    private float fieldOfViewDegree;

    /// <summary>
    /// Called after the Setup function for <see cref="FetchGamePhysicsTrainingArena"/>).
    /// </summary>
    /// <param name="helper"></param>
    public override void Setup(Janelia.IEasyMLSetupHelper helper)
    {
        // Before calling `EasyMLAgentGrounded.Setup` (as `base.Setup`) override some parameters
        // it will use.
        ChildSensorForwardDetectableTags = new List<string>()
        {
            FetchGamePhysicsTrainingArena.TAG_BOUNDARY,
            FetchGamePhysicsTrainingArena.TAG_RAMP,
            FetchGamePhysicsTrainingArena.TAG_OBSTACLE
        };
        ChildSensorForwardRayLength = GetTurfDiameter();
        BodyColor = "#4b3c39";

        base.Setup(helper);

        Transform raySensorTransform = transform.Find("RaysForward");
        if (raySensorTransform != null)
        {
            raySensorTransform.localPosition = new Vector3(0, BodyScale.y, BodyScale.z / 2);
        }

        // With timestep of 0.02, 4 stacked observations amounts to raysensor observation of the past 0.5 second.
        GameObject raySensor = GameObject.Find("RaysForward");
        RayPerceptionSensorComponent3D raySensorComponent = raySensor.GetComponent<RayPerceptionSensorComponent3D>();
        raySensorComponent.ObservationStacks = 4;

        BehaviorParameters behavior = GetComponent<BehaviorParameters>() as BehaviorParameters;
        behavior.BrainParameters.NumStackedVectorObservations = 4;
        
        moveForce = 7.0f;
    }

    /// <summary>
    /// Called every time the Agent receives an action to take. Receives the action chosen by the Agent. 
    /// It is also common to assign a reward in this method.
    /// </summary>
    /// <param name="actions">Action to take</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // The `EasyMLAgentGround` base class handles everything except the assignment of rewards.
        base.OnActionReceived(actions);

        // https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#rewards-summary--best-practices
        // "If you want the agent to finish a task quickly, it is often helpful to provide a small penalty every step
        // (-0.05) that the agent does not complete the task. In this case completion of the task should also coincide
        // with the end of the episode by calling EndEpisode() on the agent when it has accomplished its goal."
        // Also, the total penalty over all the steps should not exceed 1.

        float perStepPenalty = -1.0f / (float)MaxStep;
        AddReward(perStepPenalty);

        if (_ball != null)
        {
            float ballFetchedThreshold = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_fetched_threshold", 0.0f);
            float thresholdDistance = ballFetchedThreshold * GetTurfDiameter();
            float distanceToBall = Vector3.Distance(transform.position, _ball.transform.position);

            if (distanceToBall < thresholdDistance)
            {
                Debug.Log(transform.parent.name + " distance " + distanceToBall + " is within ball_fetched_threshold distance " + thresholdDistance);
                AddFetchedReward();
            }
        }
    }

    /// <summary>
    /// Collects vector observations from the environment.
    /// </summary>
    /// <param name="sensor">The vector sensor</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        float ballObserved = IsBallObservable() ? 1 : 0;
        if ((_ball == null) || (_ballRigidBody == null) || (ballObserved == 0))
        {
            sensor.AddObservation(new float[VectorObservationSize]);
            return;
        }

        // Indicator that encodes if the agent can see the ball with 1 indicating yes and 0 if not.
        sensor.AddObservation(ballObserved);

        // Normalize observations to [0, 1] or [-1, 1]
        // https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#normalization

        Vector3 toBall = _ball.transform.position - transform.position;
        float angleForward = SignedAngleNormalized(transform.forward, toBall);
        // One observation
        sensor.AddObservation(angleForward);

        // One observation
        // Dividing makes it a relative distance
        float turfDiameter = GetTurfDiameter();
        sensor.AddObservation(toBall.magnitude / turfDiameter);

        Vector3 ballVelocity = _ballRigidBody.velocity;

        // Angle will be 0 if the ball is moving directly away from the agent,
        // or if the ball is not moving.  Angle will be negative if the ball is moving
        // to the left of the agent's forward direction, and positive for the right.
        // Angle will be 1 (or maybe -1) if the ball is moving directly towards the agent.
        float angleVelocity = SignedAngleNormalized(transform.forward, ballVelocity);
        // One observation
        sensor.AddObservation(angleVelocity);

        // Normalize
        float agentSpeed = _agentRigidbody.velocity.magnitude / turfDiameter;
        float ballSpeed = ballVelocity.magnitude / turfDiameter;

        // But this normalization may make the values very small, so use a heuristic to increase them
        float speedScale = 4.0f;
        agentSpeed = Mathf.Clamp01(agentSpeed * speedScale);
        ballSpeed = Mathf.Clamp01(ballSpeed * speedScale);

        // One observation
        sensor.AddObservation(agentSpeed);
        // One observation
        sensor.AddObservation(ballSpeed);
    }

    /// <summary>
    /// Called on the frame when a script is enabled just before any of the Update methods 
    /// are called the first time.
    /// </summary>
    private void Start()
    {
        Debug.Log(transform.parent.name + " FetchAgent.Start");

        GameObject arena = transform.parent.gameObject;
        _ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(arena, FetchGamePhysicsTrainingArena.TAG_BALL);

        _ballRigidBody = _ball.GetComponent<Rigidbody>();

        // Set the field of view degree (single direction) from the MaxRayDegree used by the raySensor.
        GameObject raySensor = GameObject.Find("RaysForward");
        fieldOfViewDegree = raySensor != null ? raySensor.GetComponent<RayPerceptionSensorComponent3D>().MaxRayDegrees : 70;
        
        // Make the agent camera view be consistent with the actual field of view set.
        Camera agentCamera = GameObject.Find("AgentCamera").GetComponent<Camera>();
        agentCamera.fieldOfView = fieldOfViewDegree / agentCamera.aspect * 2;
    }

    /// <summary>
    /// Called when the agent collides with another scene object.
    /// </summary>
    /// <param name="collision">The collision info</param>
    private void OnCollisionEnter(Collision collision)
    {
        Collider c = collision.collider;
        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_BALL))
        {
            // Disable any collision response that might put the ball or agent in a bad position.
            _ballRigidBody.Sleep();
            _agentRigidbody.Sleep();

            if (trainingMode)
            {
                AddFetchedReward();
            }
        }

        // TODO: There is no penalty for a collision with the obstacle, to keep training from being
        // too difficult.  Is this approach right?  Should there be no penalty for any collision?
        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_BOUNDARY) || c.CompareTag(FetchGamePhysicsTrainingArena.TAG_RAMP))
        {
            if (trainingMode)
            {
                Debug.Log(transform.parent.name + " adding collision penalty");
                AddReward(-0.5f);
            }
        }
    }

    // private void OnCollisionStay(Collision collision)
    // {
    //     if (collision.collider.CompareTag(FetchGamePhysicsTrainingArena.TAG_OBSTACLE))
    //     {
    //         // When colliding with the obstacle, hack the movement force direction to make
    //         // the agent slide along the obstacle instead of getting stuck on it.  The algorithm
    //         // assumes the obstacle is a plane.
    //         Vector3 v = Vector3.ProjectOnPlane(transform.forward, collision.gameObject.transform.right);
    //         v.Normalize();
    //         _moveForwardDirection = v;
    //     }
    // }

    // private void OnCollisionExit(Collision collision)
    // {
    //     if (collision.collider.CompareTag(FetchGamePhysicsTrainingArena.TAG_OBSTACLE))
    //     {
    //         // When there is no more collison with the obstacle, go back to the standard 
    //         // movement force direction.
    //         _moveForwardDirection = Vector3.zero;
    //     }
    // }

    private void AddFetchedReward()
    {
        Vector3 toBall = (_ball.transform.position - transform.position);
        float speed_bonus_proportion = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_bonus", 0.8f);
        float orientation_bonus = 0.5f * (1 - speed_bonus_proportion) * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, toBall.normalized));
        float speed_bonus = 0.5f * speed_bonus_proportion * (1 - StepCount / MaxStep);
        AddReward(0.5f + orientation_bonus + speed_bonus);
        Debug.Log(transform.parent.name + " successfully complete task with reward: " + GetCumulativeReward());
        EndEpisode();
    }

    private float GetTurfDiameter()
    {
        FetchGamePhysicsTrainingArena fetchArena = GetComponentInParent<FetchGamePhysicsTrainingArena>();
        return (fetchArena != null) ? 2.0f * fetchArena.TurfRadius : 1.0f;
    }

    private float SignedAngleNormalized(Vector3 a, Vector3 b)
    {
        return Vector3.SignedAngle(a, b, transform.up) / 180;
        // // [0, 180] for left or right
        // float angleBetween = Vector3.Angle(a.normalized, b.normalized);
        // // Negative for left
        // Vector3 cross = Vector3.Cross(a, b);
        // float sign = Mathf.Sign(Vector3.Dot(cross, transform.up));
        // return sign * angleBetween / 180;
    }

    private bool IsBallObservable()
    {
        Vector3 rayInit = transform.TransformPoint(BodyScale / 3);
        Vector3 toBall = _ball.transform.position - rayInit;
        float distance = toBall.magnitude - _ball.transform.localScale.x;
        float angleToBall = Vector3.SignedAngle(transform.forward, toBall, transform.up);
        bool atFront = angleToBall < fieldOfViewDegree && angleToBall > -fieldOfViewDegree;
        RaycastHit hit;
        bool blocked = Physics.Raycast(rayInit, toBall, out hit, distance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
        if (blocked) Debug.Log("Raycast hit " + hit.collider);
        return atFront && !blocked;
    }
}
