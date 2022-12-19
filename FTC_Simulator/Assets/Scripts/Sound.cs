using UnityEngine.Audio;
using UnityEngine;

[System.Serializable]
public class Sound
{
    [SerializeField]
    public string name;
    
    // The private values are set in the editor and only applied on awake. If you want to change
    // the value later, change the underscored parameter
    [SerializeField]
    private AudioClip clip;
    public AudioClip _clip { 
        get { return (source == null) ? clip : source.clip; } 
        set { if (source == null) { clip = value; } else { source.clip = value; } } 
    }

    [SerializeField]
    private float volume;
    private float initial_volume = 0f;
 
    public float _volume { 
        get {
            if (source != null)
                return source.volume;
            else
                return volume;
        } set {
            if (source)
            { source.volume = value; }
            else
            { volume = value; }
        } 
    }

    public float get_init_volume()
    {
        return initial_volume;
    }

    [SerializeField]
    private float pitch;
    public float _pitch { 
        get { return (source == null) ? pitch : source.pitch; }
        set { if (source == null) { pitch = value; } else { source.pitch = value; } } 
    }

    [SerializeField]
    private float spatialBlend;
    public float _spatialBlend { 
        get { return (source == null) ? spatialBlend : source.spatialBlend; }
        set { if (source == null) { spatialBlend = value; } else { source.spatialBlend = value; } }
    }

    [SerializeField]
    private bool loop;
    public bool _loop { 
        get { return (source == null) ? loop : source.loop; }
        set { if (source == null) { loop = value; } else { source.loop = value; } }
    }

    /// <summary>
    /// Set to null if you want it to play from the transform it's on
    /// </summary>
    public Transform sourceLocation = null;

    public bool isPlaying => source.isPlaying;
    public float time { 
        get { return source.time; } 
        set {source.time = value; } 
    }

    [HideInInspector]
    private AudioSource source;

    public bool started = false; // set and managed by audiomanager


    public void Play() {
        source.Play();
    }

    public void Stop() {
        source.Stop();
    }

    public Sound Clone() {
        Sound ret = new Sound();
        ret.name = name;
        ret.clip = clip;
        ret.volume = volume;
        ret.pitch = pitch;
        ret.loop = loop;
        ret.source = source;
        ret.spatialBlend = spatialBlend;
        return ret;
    }

    public Sound() {
        

    }


    // here we instantiate our audio source under the correct GO
    public void Init(AudioManager myManager) {
        if (sourceLocation == null) {
            sourceLocation = myManager.transform;
        }

        source = sourceLocation.gameObject.AddComponent<AudioSource>();
        source.playOnAwake = false;
        _clip = clip;
        _volume = volume;
        initial_volume = volume;
        _pitch = pitch;
        _loop = loop;
        _spatialBlend = spatialBlend;
        source.minDistance = GLOBALS.SOUND_MIN_DISTANCE;
        source.maxDistance = GLOBALS.SOUND_MAX_DISTANCE;
        source.rolloffMode = AudioRolloffMode.Linear;
    }
}
