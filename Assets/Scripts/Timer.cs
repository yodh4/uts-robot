using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class Timer : MonoBehaviour
{
    public TMP_Text timerText;
    public float currTime = 0f;

    public static Timer instance;

    void Awake(){
        if(instance == null){
            instance = this;
        } else {
            Destroy(gameObject);
        }
    }

    void Start(){
        StartCoroutine(StartTimer());
    }

    IEnumerator StartTimer(){
        while(true){
            yield return new WaitForSeconds(1);
            currTime++;

            float minutes = Mathf.Floor(currTime / 60);
            float seconds = currTime % 60;

            timerText.text = "Time elapsed: " + minutes.ToString("00") + ":" + seconds.ToString("00");
        }
    }

    public void StopTimer(){
        StopAllCoroutines();
    }
}
