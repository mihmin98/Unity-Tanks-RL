using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class StaticTargetController : MonoBehaviour
{
    public GameObject spawnPointsParent;
    public int timesToTouchUntilRandomRespawn = 0;

    [Header("RNG")]
    public bool useSeededRNG = false;
    public Int32 randomSeed = 0;

    private System.Random rng;

    void Start()
    {
        spawnPointsParent = transform.parent.Find("SpawnPoints").gameObject;

        if (useSeededRNG)
        {
            rng = new System.Random(randomSeed);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        PlayerAgent_MoveToStaticTarget agentStatic = other.GetComponent<PlayerAgent_MoveToStaticTarget>();
        PlayerAgent_MoveToMovingTarget agentMoving = other.GetComponent<PlayerAgent_MoveToMovingTarget>();

        if (agentStatic != null || agentMoving != null)
        {
            --timesToTouchUntilRandomRespawn;
            if (timesToTouchUntilRandomRespawn <= 0) // Done all the touches
            {
                if (agentStatic != null)
                    agentStatic.TouchedObjective(false);
                if (agentMoving != null)
                    agentMoving.TouchedObjective(false);
                transform.position = GetRandomSpawnPosition();
            }
            else
            {
                if (agentStatic != null)
                    agentStatic.TouchedObjective(true);
                if (agentMoving != null)
                    agentMoving.TouchedObjective(true);
            }
        }
    }

    private Vector3 GetRandomSpawnPosition()
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
}
