using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;

public class MoveToMovingTargetTestController : MonoBehaviour
{
    private float elapsedTime = 0f;

    public GameObject agentTank;
    private Vector3 agentInitialPosition;
    private Quaternion agentInitialRotation;
    public MovingTargetController targetController;
    private PlayerAgent_MoveToStaticTarget playerAgentStatic;
    private PlayerAgent_MoveToMovingTarget playerAgentMoving;
    private BehaviorParameters behaviorParameters;

    public int timesToReachTarget = 50;
    private int currentTimesReachedTarget = 0;

    public float timeScale = 1f;



    void Start()
    {
        agentInitialPosition = agentTank.transform.position;
        agentInitialRotation = agentTank.transform.rotation;

        playerAgentStatic = agentTank.GetComponent<PlayerAgent_MoveToStaticTarget>();
        playerAgentMoving = agentTank.GetComponent<PlayerAgent_MoveToMovingTarget>();
        behaviorParameters = agentTank.GetComponent<BehaviorParameters>();
        elapsedTime = 0f;

        Time.timeScale = timeScale;
    }

    void Update()
    {
        elapsedTime += Time.deltaTime;

        if (playerAgentStatic != null)
        {
            if (currentTimesReachedTarget != playerAgentStatic.timesReachedObjective)
            {
                currentTimesReachedTarget = playerAgentStatic.timesReachedObjective;
                targetController.ResetSeededRNG(currentTimesReachedTarget);
                targetController.GetNewPositionAndDestination();
                agentTank.transform.position = agentInitialPosition;
                agentTank.transform.rotation = agentInitialRotation;
            }

            if (playerAgentStatic.timesReachedObjective == timesToReachTarget)
            {
                Debug.Log("Elapsed time: " + elapsedTime + "\nModel Name: " + behaviorParameters.Model.name);
                EditorApplication.ExitPlaymode();
            }
        }
        else if (playerAgentMoving != null)
        {
            if (currentTimesReachedTarget != playerAgentMoving.timesReachedObjective)
            {
                currentTimesReachedTarget = playerAgentMoving.timesReachedObjective;
                targetController.ResetSeededRNG(currentTimesReachedTarget);
                targetController.GetNewPositionAndDestination();
                agentTank.transform.position = agentInitialPosition;
                agentTank.transform.rotation = agentInitialRotation;
            }

            if (playerAgentMoving.timesReachedObjective == timesToReachTarget)
            {
                Debug.Log("Elapsed time: " + elapsedTime + "\nModel Name: " + behaviorParameters.Model.name);
                EditorApplication.ExitPlaymode();
            }
        }
    }
}
