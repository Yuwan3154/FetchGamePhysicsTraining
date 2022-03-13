using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// A trainable agent for the fetch game:
/// <seealso href="https://github.com/JohnsonLabJanelia/FetchGamePhysics/tree/main/FetchArenaProject"/>
/// </summary>
public class FetchGamePhysicsTrainingAgent : Janelia.EasyMLAgentGrounded
{
    public override string BehaviorName { get; protected set; } = "FetchGamePhysics";

    public override int VectorObservationSize { get; protected set; } = 11;

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
            FetchGamePhysicsTrainingArena.TAG_RAMP
        };
        ChildSensorForwardRayLength = GetTurfDiameter();
        BodyColor = "#4b3c39";

        base.Setup(helper);

        moveForce = 15.0f;
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
        if ((_ball == null) || (_ballRigidBody == null))
        {
            sensor.AddObservation(new float[VectorObservationSize]);
            return;
        }

        // One observation
        float agentRotation = transform.localEulerAngles.y / 360;
        sensor.AddObservation(agentRotation);

        Vector3 toBall = _ball.transform.position - transform.position;
        // Three observations (the vector components)
        // Normalizing makes it just a direction
        sensor.AddObservation(toBall.normalized);

        // One observation
        // Dividing makes it a relative distance
        sensor.AddObservation(toBall.magnitude / GetTurfDiameter());

        Vector3 agentVelocity = _agentRigidbody.velocity;
        // Three observations (the vector components)
        sensor.AddObservation(agentVelocity.normalized);

        Vector3 ballVelocity = _ballRigidBody.velocity;
        // Three observations (the vector components)
        sensor.AddObservation(ballVelocity.normalized);

        // Nine total observations (1 + 3 + 1 + 3 + 3)
        Debug.Assert(VectorObservationSize == 11, "Incorrect observation count");
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
        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_BOUNDARY) || c.CompareTag(FetchGamePhysicsTrainingArena.TAG_RAMP))
        {
            if (trainingMode)
            {
                Debug.Log(transform.parent.name + " adding collision penalty");
                AddReward(-0.5f);
            }
        }
    }

    private void AddFetchedReward()
    {
        Vector3 toBall = (_ball.transform.position - transform.position);
        float bonus = 0.5f * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, toBall.normalized));
        AddReward(0.5f + bonus);
        EndEpisode();
    }

    private float GetTurfDiameter()
    {
        FetchGamePhysicsTrainingArena fetchArena = GetComponentInParent<FetchGamePhysicsTrainingArena>();
        return (fetchArena != null) ? 2.0f * fetchArena.TurfRadius : 1.0f;
    }
}
