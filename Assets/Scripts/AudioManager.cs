using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AudioManager : MonoBehaviour
{
    public AudioSource mSource;
    public AudioClip SFXDefused, SFXFinished;

    public static AudioManager instance;

    void Awake(){
        if(instance == null){
            instance = this;
        } else {
            Destroy(gameObject);
        }
    }

    public void PlayDefusedSFX(){
        mSource.PlayOneShot(SFXDefused);
    }

    public void PlayFinishedSFX(){
        mSource.PlayOneShot(SFXFinished);
    }
}
