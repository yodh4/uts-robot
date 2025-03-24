using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bomb : MonoBehaviour
{
    public int locationArea;

    void OnTriggerEnter(Collider other) {
        if(other.CompareTag("Player")){
            BombCounter.instance.DefuseBomb();
            gameObject.SetActive(false);
        }
    }
}
