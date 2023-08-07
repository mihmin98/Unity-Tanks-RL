using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;

public class MoveToStaticTargetTestController : MonoBehaviour
{
    private float elapsedTime = 0f;

    public GameObject agentTank;
    private PlayerAgent_MoveToStaticTarget playerAgentStatic;
    private PlayerAgent_MoveToMovingTarget playerAgentMoving;
    private BehaviorParameters behaviorParameters;

    public int timesToReachTarget = 50;

    public float timeScale = 1f;

    // Start is called before the first frame update
    void Start()
    {
        playerAgentStatic = agentTank.GetComponent<PlayerAgent_MoveToStaticTarget>();
        playerAgentMoving = agentTank.GetComponent<PlayerAgent_MoveToMovingTarget>();
        behaviorParameters = agentTank.GetComponent<BehaviorParameters>();
        elapsedTime = 0f;

        Time.timeScale = timeScale;
    }

    // Update is called once per frame
    void Update()
    {
        elapsedTime += Time.deltaTime;

        if (playerAgentStatic != null)
        {
            if (playerAgentStatic.timesReachedObjective == timesToReachTarget)
            {
                Debug.Log("Elapsed time: " + elapsedTime + "\nModel Name: " + behaviorParameters.Model.name);
                EditorApplication.ExitPlaymode();
            }
        } else if (playerAgentMoving != null)
        {
            if (playerAgentMoving.timesReachedObjective == timesToReachTarget)
            {
                Debug.Log("Elapsed time: " + elapsedTime + "\nModel Name: " + behaviorParameters.Model.name);
                EditorApplication.ExitPlaymode();
            }
        }
    }
}
