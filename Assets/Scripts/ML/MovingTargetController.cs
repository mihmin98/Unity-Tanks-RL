using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class MovingTargetController : MonoBehaviour
{
    public float distanceUntilReachDestination = 1f;
    private NavMeshAgent navMeshAgent;
    private Transform destination;
    private GameObject spawnPointsParent;

    [Header("RNG")]
    public bool useSeededRNG = false;
    public Int32 randomSeed = 0;

    private System.Random rng;

    private void Start()
    {
        spawnPointsParent = transform.parent.Find("SpawnPoints").gameObject;

        if (useSeededRNG)
        {
            rng = new System.Random(randomSeed);
        }

        navMeshAgent = GetComponent<NavMeshAgent>();
        navMeshAgent.autoBraking = false;
        navMeshAgent.destination = GetRandomSpawnPoint();
    }

    private void Update()
    {
        if (!navMeshAgent.pathPending && navMeshAgent.remainingDistance < distanceUntilReachDestination)
        {
            navMeshAgent.destination = GetRandomSpawnPoint();
        }
    }

    private Vector3 GetRandomSpawnPoint()
    {
        if (spawnPointsParent == null)
        {
            return Vector3.zero;
        }

        int numSpawnPoints = spawnPointsParent.transform.childCount;

        Transform[] spawnpoints = new Transform[numSpawnPoints];
        for (int i = 0; i < numSpawnPoints; i++)
        {
            spawnpoints[i] = spawnPointsParent.transform.GetChild(i);
        }

        Transform randomTransform;

        do
        {
            int randIdx = useSeededRNG ?
                rng.Next(0, numSpawnPoints) :
                UnityEngine.Random.Range(0, numSpawnPoints);

            randomTransform = spawnpoints[randIdx];
        } while (Vector3.Distance(transform.position, randomTransform.position) < 10f);

        return randomTransform.position;
    }

    private void OnTriggerEnter(Collider other)
    {
        PlayerAgent_MoveToStaticTarget agentStatic = other.GetComponent<PlayerAgent_MoveToStaticTarget>();
        PlayerAgent_MoveToMovingTarget agentMoving = other.GetComponent<PlayerAgent_MoveToMovingTarget>();
        if (agentStatic != null || agentMoving != null)
        {
            if (agentStatic != null)
                agentStatic.TouchedObjective(false);
            if (agentMoving != null)
                agentMoving.TouchedObjective(false);
            transform.position = GetRandomSpawnPoint();
            navMeshAgent.destination = GetRandomSpawnPoint();
        }
    }

    public void ResetSeededRNG(Int32 seed)
    {
        rng = new System.Random(seed);
    }

    public void GetNewPositionAndDestination()
    {
        transform.position = GetRandomSpawnPoint();
        navMeshAgent.destination = GetRandomSpawnPoint();
    }
}
