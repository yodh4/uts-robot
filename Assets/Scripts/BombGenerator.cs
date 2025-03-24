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
    }
}
