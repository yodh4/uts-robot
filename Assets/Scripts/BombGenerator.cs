using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BombGenerator : MonoBehaviour
{
    public List<GameObject> bombA, bombB, bombC;

    void Start(){
        GameObject[] bombs = GameObject.FindGameObjectsWithTag("Bombs");

        foreach(GameObject bomb in bombs){
            Bomb bombScript = bomb.GetComponent<Bomb>();

            if(bombScript.locationArea == 1){
                bombA.Add(bomb);
            } else if(bombScript.locationArea == 2){
                bombB.Add(bomb);
            } else if(bombScript.locationArea == 3){
                bombC.Add(bomb);
            }
        }
    
        // Randomize the bombs.
        RandomizeBombs(bombA);
        RandomizeBombs(bombB);
        RandomizeBombs(bombC);
    }

    void RandomizeBombs(List<GameObject> bombs){
        int randomIndex = Random.Range(0, bombs.Count);

        foreach(GameObject bomb in bombs){
            bomb.SetActive(false);
        }

        bombs[randomIndex].SetActive(true);
    }
}
