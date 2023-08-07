using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using System;
using UnityEngine.AI;

public class PlayerAgent_MoveToMovingTarget : Agent
{
    public GameObject gymObject;
    public GameObject target;
    public bool useManualInput = false;
    public bool useCamera = true;

    [Header("Raycasts")]
    public int numRays = 32;
    public float raycastHeight = 0.5f;
    private readonly List<Vector3> rayDirections = new List<Vector3>();
    private List<(RaycastHit, bool)> raycastHits = null;

    [Header("Enable/Disable Observations")]
    public bool useAgentPositionObservation = false;
    public bool useTargetPositionObservation = false;
    public bool useTargetVelocityObservation = true;

    [Header("Penalties")]
    public float constantPenalty = -0.005f;
    public float movingAwayFromTargetPenalty = -0.025f;
    public float suddenDirectionChangePenalty = -0.01f;
    public float goingIntoWallPenaltyPerRay = -0.002f;
    public float notMovingPenalty = -0.05f;
    public int numStepsUntilFailForNotMoving = 100;
    private int currentStepsNotMoving = 0;

    [Header("Rewards")]
    public float movingTowardsTargetReward = 0.001f;
    public float objectiveReward = 10f;

    private float[] discreteMovmentArray = { -1f, -0.5f, 0f, 0.5f, 1f };

    private TankMovement tankMovement;
    private Vector3 initialPosition;
    private Vector3 previousPosition;
    private Vector3 previousVelocity;

    private NavMeshAgent targetNavMeshAgent;

    private CameraControl cameraControl;

    [Header("Counters")]
    public int timesReachedObjective = 0;

    public override void Initialize()
    {
        // Disable max steps if not training
        if (!Academy.Instance.IsCommunicatorOn)
        {
            MaxStep = 0;
        }

        gymObject = transform.parent.gameObject;
        target = gymObject.transform.Find("Target").gameObject;
        targetNavMeshAgent = target.GetComponent<NavMeshAgent>();

        tankMovement = GetComponent<TankMovement>();
        tankMovement.SetManualInput(useManualInput);

        float stepSize = Mathf.Deg2Rad * 360 / numRays;
        for (int i = 0; i < numRays; ++i)
        {
            float currentAngle = i * stepSize;
            Vector3 ray = new Vector3(Mathf.Cos(currentAngle), 0, Mathf.Sin(currentAngle));
            rayDirections.Add(ray);
        }

        initialPosition = transform.position;

        SetupMLAgents();

        if (useCamera)
        {
            cameraControl = gymObject.transform.Find("CameraRig").gameObject.GetComponent<CameraControl>();
            Array.Resize(ref cameraControl.m_Targets, cameraControl.m_Targets.Length + 2);
            cameraControl.m_Targets[cameraControl.m_Targets.Length - 2] = target.transform;
            cameraControl.m_Targets[cameraControl.m_Targets.Length - 1] = transform;
        }
    }

    private void SetupMLAgents()
    {
        // Behavior Parameters
        BehaviorParameters behaviorParameters = GetComponent<BehaviorParameters>();
        ActionSpec actionSpec = new ActionSpec(0, new Int32[] { discreteMovmentArray.Length, discreteMovmentArray.Length });
        behaviorParameters.BrainParameters.ActionSpec = actionSpec;

        // Observation Size
        int observationSize = numRays * 2; // for each ray add distance and hit layer
        if (useTargetPositionObservation)
            observationSize += 3; // target position
        if (useAgentPositionObservation)
            observationSize += 3; // agent position
        observationSize += 3; // forward vector
        observationSize += 3; // direction towards target
        observationSize += 1; // angle towards target
        observationSize += 1; // distance to target
        observationSize += 3; // velocity
        observationSize += 1; // velocity angle compared to forward
        observationSize += 1; // velocity magnitude
        if (useTargetVelocityObservation)
            observationSize += 3; // target velocity

        behaviorParameters.BrainParameters.VectorObservationSize = observationSize;
    }

    public override void OnEpisodeBegin()
    {
        transform.position = initialPosition;
        previousPosition = transform.position;
        currentStepsNotMoving = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // target position
        if (useTargetPositionObservation)
            sensor.AddObservation(target.transform.position);

        // agent position
        if (useAgentPositionObservation)
            sensor.AddObservation(transform.position);

        // forward vector
        sensor.AddObservation(transform.forward);

        // direction towards target
        Vector3 directionTowardsTarget = target.transform.position - transform.position;
        sensor.AddObservation(directionTowardsTarget.normalized);

        // angle towards target
        float angleTowardsTarget = Vector3.SignedAngle(directionTowardsTarget, tankMovement.m_movingAvgVel, Vector3.up) / 180f;
        sensor.AddObservation(angleTowardsTarget);

        // distance to target
        sensor.AddObservation(Vector3.Distance(transform.position, target.transform.position));

        // velocity
        sensor.AddObservation(tankMovement.m_movingAvgVel.normalized);

        // velocity angle compared to forward
        sensor.AddObservation(Vector3.SignedAngle(transform.forward, tankMovement.m_movingAvgVel, Vector3.up) / 180f);

        // velocity magnitude
        sensor.AddObservation(tankMovement.m_movingAvgVel.magnitude);

        // target velocity
        if (useTargetVelocityObservation)
        {
            if (targetNavMeshAgent != null)
                sensor.AddObservation(targetNavMeshAgent.velocity.normalized);
            else
                sensor.AddObservation(Vector3.zero);
        }

        // raycast distance and layers
        foreach (var hitTuple in (raycastHits != null ? raycastHits : GetRaycastHits()))
        {
            RaycastHit hitInfo = hitTuple.Item1;
            bool hit = hitTuple.Item2;

            if (hit)
            {
                sensor.AddObservation(hitInfo.distance);
                sensor.AddObservation(hitInfo.collider.gameObject.layer);
            }
            else
            {
                sensor.AddObservation(10000f);
                sensor.AddObservation(0);
            }
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int discreteMovementIdx = actions.DiscreteActions[0];
        int discreteTurnIdx = actions.DiscreteActions[1];

        float movementValue = discreteMovmentArray[discreteMovementIdx];
        float turnValue = discreteMovmentArray[discreteTurnIdx];

        if (!useManualInput)
        {
            tankMovement.SetMovementValue(movementValue);
            tankMovement.SetTurningValue(turnValue);
        }
    }

    void Update()
    {
        DrawDebugRays();

        if (useCamera)
        {
            cameraControl.m_Targets[cameraControl.m_Targets.Length - 2] = target.transform;
        }
    }

    private void FixedUpdate()
    {
        raycastHits = GetRaycastHits();

        UpdatePenalties();
        UpdateRewards();

        previousVelocity = tankMovement.m_movingAvgVel;
        previousPosition = transform.position;
    }

    private void UpdatePenalties()
    {
        // not moving penalty
        if (tankMovement.m_movingAvgVel.magnitude < 0.05f)
        {
            ++currentStepsNotMoving;
            AddReward(notMovingPenalty);
            if (currentStepsNotMoving >= numStepsUntilFailForNotMoving)
            {
                Debug.Log(gymObject.transform.name + ": Reset not moving");
                AddReward(notMovingPenalty * 1000);
                EndEpisode();
            }
        }
        else
        {
            currentStepsNotMoving = 0;
        }

        // constant penalty
        AddReward(constantPenalty);

        float angle, anglePenalty;

        // penalty based on direction of movment
        Vector3 directionTowardsTarget = target.transform.position - transform.position;
        angle = Mathf.Abs(Vector3.SignedAngle(directionTowardsTarget, tankMovement.m_movingAvgVel, Vector3.up) / 180f);
        anglePenalty = (angle / 180f) * movingAwayFromTargetPenalty;
        AddReward(anglePenalty);

        // penalty if going backwards
        angle = Mathf.Abs(Vector3.Angle(transform.forward, tankMovement.m_movingAvgVel));
        if (angle > 90f)
        {
            anglePenalty = (angle / 180f) * constantPenalty;
            AddReward(anglePenalty);
        }

        // penalty based on angle change
        angle = Mathf.Abs(Vector3.Angle(previousVelocity, tankMovement.m_movingAvgVel));
        anglePenalty = (angle / 180f) * suddenDirectionChangePenalty;
        AddReward(anglePenalty);

        // penalty for getting too close to walls
        LayerMask layerMask = LayerMask.GetMask("Default", "Ground");
        foreach (var hitTuple in raycastHits)
        {
            RaycastHit hitInfo = hitTuple.Item1;
            int layer = 0;
            bool hit = hitTuple.Item2;
            if (hit)
            {
                layer = hitInfo.collider.gameObject.layer;
            }

            if (hit && InLayerMask(layer, layerMask))
            {
                if (hitInfo.distance < 3f)
                {
                    float penalty = goingIntoWallPenaltyPerRay / hitInfo.distance;
                    AddReward(penalty);
                }
            }
        }
    }

    private void UpdateRewards()
    {
        float angle, angleReward;

        // reward based on distance to target and direction of movment
        Vector3 directionTowardsTarget = target.transform.position - transform.position;
        angle = Mathf.Abs(Vector3.SignedAngle(directionTowardsTarget, tankMovement.m_movingAvgVel, Vector3.up) / 180f);
        float distanceToTarget = Vector3.Distance(transform.position, target.transform.position);
        if (distanceToTarget >= 1f)
        {
            angleReward = (1 - (angle / 180f)) * movingTowardsTargetReward / distanceToTarget;
            AddReward(angleReward);
        }

        // reward if close to target
        LayerMask layerMask = LayerMask.GetMask("Targets");
        foreach (var hitTuple in raycastHits)
        {
            RaycastHit hitInfo = hitTuple.Item1;
            int layer = 0;
            bool hit = hitTuple.Item2;
            if (hit)
            {
                layer = hitInfo.collider.gameObject.layer;
            }

            if (hit)
            {
                if (InLayerMask(layer, layerMask) && hitInfo.distance >= 1f)
                {
                    float reward = movingTowardsTargetReward / (numRays * hitInfo.distance);
                    AddReward(reward);
                }
            }
        }
    }

    public void TouchedObjective(bool endEpisode = false)
    {
        ++timesReachedObjective;
        Debug.Log(gymObject.transform.name +  ": Touched Objective (total " + timesReachedObjective + ")");
        AddReward(objectiveReward);

        if (endEpisode)
        {
            EndEpisode();
        }
    }

    private List<(RaycastHit, bool)> GetRaycastHits()
    {
        List<(RaycastHit, bool)> raycastHits = new List<(RaycastHit, bool)>();
        foreach (Vector3 rayDir in rayDirections)
        {
            bool hit = Physics.Raycast(transform.position + transform.up * raycastHeight, transform.TransformDirection(rayDir), out RaycastHit hitInfo, Mathf.Infinity);
            raycastHits.Add((hitInfo, hit));
        }

        return raycastHits;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

    }
    private bool InLayerMask(int layer, LayerMask layerMask)
    {
        return layerMask == (layerMask | (1 << layer));
    }

    public void DrawDebugRays(float length = 100f)
    {
        foreach (Vector3 rayDir in rayDirections)
        {
            Debug.DrawRay(transform.position + transform.up * raycastHeight, rayDir * length, Color.magenta);
        }
    }
}
