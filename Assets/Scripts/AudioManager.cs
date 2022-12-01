using UnityEngine.Audio;
using static UnityEngine.Debug;
using System;
using System.Threading;
using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Audio Manager
//
// During single player mode, it manages sound and allows mutliple sounds to be mixed together
//
// During Multiplayer mode:
//   1) A game element needs to be attached so that a unique ID can be extracted. It also needs to be tagged as "GameElement".
//      This has the side effect of potentially being moved by the server (not used at this time). A bandwidth helper can be attached to prevent any real data from being sent. during fieldElement packet
//   2) The server will have this class register that a sound needs to be played under it's ID
//   3) The client side will only accept play events from packets coming in, and will ignore in game generated sounds.
//   By default, audio manager game element ID's should be in the 500's if not attached to a normal game element (e.g. main timer audio, etc..)
public class AudioManager : MonoBehaviour
{
    //holds all the sounds that could be played by the frc robot
    public List<Sound> sounds;

    private Dictionary<string, Sound> sounds_indexed = new Dictionary<string, Sound>();
    public ArrayList allRamps = new ArrayList();

    struct Ramp {
        public Sound mySound;
        public float timeStartedRamp;
        public float rampTime;
        public float initialVolume;
        public bool up;

        public Ramp(Sound s, float startTime, float totalTime, float initialVolume, bool up) {
            mySound = s;
            timeStartedRamp = startTime;
            rampTime = totalTime;
            this.initialVolume = initialVolume;
            this.up = up;
        }

    }

    void Awake() {
        // Go through all our sounds
        foreach(Sound currsound in sounds)
        {
            // Add it to our indexer for ease of access
            sounds_indexed.Add(currsound.name, currsound);

            // Initialize the sound
            currsound.Init(this);
        }
    }

    public void AddSound(Sound newsound)
    {
        // Make sure there is a sound
        if( newsound==null ) { return; }

        // Add it to our easy-access index and initialize the sound.
        sounds_indexed.Add(newsound.name, newsound);
        newsound.Init(this);

        // Add it to our list as well... not that is really ever gets used
        sounds.Add(newsound);
    }


    // Play a sound in game
    int id = 0;

    public bool Play(string name, float crossFadeTime = 0, float volume = -1f, bool server_request = false, float pitch = -1f)
    {        
        // If sound doesn't exist, exit
        Sound s = findSound(name);
        if (s == null)
        {
            return false;
        }
        s.started = true;

        // Determine our Id if not found already
        // Find gameElement if not found
        if (id == 0)
        {
            gameElement mygameelement = GetComponent<gameElement>();
            RobotID myrobotid = GetComponent<RobotID>();
            if (mygameelement) { id = mygameelement.id; }
            else if (myrobotid) { id = -1 * myrobotid.id; }
        }


        // If this is server mode, send a packet id
        // Only if this is not a message marked as server_request
        if ( GLOBALS.SERVER_MODE && !server_request)
        {
            // If we don't have an ID, exit
            if( id == 0 ) { return false; }

            // Get the sound volume

            if( volume < 0 )
            {
                volume = s.get_init_volume(); ;
            }

            // Register this sound under our id
            GLOBALS.topserver.playSound(id, name, crossFadeTime, volume, pitch);
        }

        // Now play the sound
        if (server_request || !GLOBALS.CLIENT_MODE) { return PlayCore(name, crossFadeTime, volume, pitch); }
        return true;

    }

    private bool PlayCore(string name, float crossFadeTime = 0f, float volume = -1f, float pitch = -1f)
    {
        // Don't play on headless
        if(GLOBALS.HEADLESS_MODE) { return false; }

        if (!GLOBALS.AUDIO) { return false; }
        if (!GLOBALS.ROBOTAUDIO && (id < 0 )) { return false; }

        Sound s = findSound(name);

        // If sound not found, what the heck?
        if( s == null)
        {
            UnityEngine.Debug.Log("Sound " + name + " not found!");
            return false;
        }

        if (volume < 0)
        { volume = s.get_init_volume(); }
       
        s._volume = volume; 

        if( pitch > 0)
        {
            s._pitch = pitch;
        }

        //error if we can't find the sound
        if (s == null)
        {
            return false;
        }

        // Remove old ramps of this sound
        for (int i = allRamps.Count-1; i >= 0; i--)
        {
            if (((Ramp)allRamps[i]).mySound.name == name)
            {
                allRamps.Remove(allRamps[i]);
            }
        }

        // Create new ramp
        allRamps.Add(new Ramp(s, Time.time, crossFadeTime, s._volume, true));
        
        // Start with 0 volume until Ramp will udpate it to correct volume
        // But if cross-fase = 0 then just start at target volume
        if( crossFadeTime != 0)
        {
            s._volume = 0;
        }
        
        s.Play();
        return true;
    }


    public bool Stop(string name, float crossFadeTime, bool server_request = false) {
        // If sound doesn't exist, exit
        Sound s = findSound(name);
        if (s == null)
        {
            return false;
        }
        s.started = false;


        foreach (Ramp r in allRamps.ToArray())
        {
            if (r.mySound.name == name)
            {
                if (!r.up)
                {
                    return false;
                } 
                else
                {
                    if( allRamps.Contains(r))
                    {
                        r.mySound._volume = r.initialVolume;
                        allRamps.Remove(r);
                    }                   
                }
            }
        }

        // If this is a client request (not server), then don't do it
        if (!server_request && GLOBALS.CLIENT_MODE) { return true; }



        // If this is server mode, send a packet id
        if (GLOBALS.SERVER_MODE)
        {
            // Find gameElement if not found
            if (id == 0)
            {
                gameElement mygameelement = GetComponent<gameElement>();
                RobotID myrobotid = GetComponent<RobotID>();
                if (mygameelement) { id = mygameelement.id; }
                else if (myrobotid) { id = -1 * myrobotid.id; }
            }            

            // Register this sound under our id
            GLOBALS.topserver.stopSound(id, name, crossFadeTime);
        }

        allRamps.Add(new Ramp(s,Time.time, crossFadeTime, s._volume, false));
        return true;
    }

    //use this to loop sounds
    public bool restartSoundCrossFade(string name, float crossFadeTime) {
        Sound s1 = findSound(name);
        if(s1 == null) { return false; }

        s1.time = 0;
        /*
        //attempt to find the sound that matches this one
        Sound s2 = findSound(name + "2");

        bool startSecondSound = true;
        //if we didn't find it, make one
        if(s2 == null) {
            s2 = s1.Clone();
            s2.name = name + "2";
            sounds.Add(s2);
            s2.Init(this);
        } else{
            if(s2._volume > 0.01) {
                startSecondSound = false;
            }
        }

        if(startSecondSound) {
            Play(s2.name,crossFadeTime);
            Stop(s1.name,crossFadeTime);
        }else {
            Play(s1.name,crossFadeTime);
            Stop(s2.name,crossFadeTime);
        }*/
        return true;
    }

    

    void Update() 
    {
        // **** DEBUG INFORMATION 
        //allRamps_count = allRamps.Count;
        //allRamps_names.Clear();
        //for (int i = 0; i < allRamps.Count; i++)
        //{
        //    allRamps_names.Add(((Ramp) allRamps[i]).mySound.name );
        //}
           // **** END DEBUG ****

    Ramp ramp;
        for (int i = 0; i < allRamps.Count; i ++) {
            ramp = (Ramp) allRamps[i];
            float elapsedSinceStart = Time.time - ramp.timeStartedRamp;

            if(ramp.up) {
                if (ramp.rampTime > 0)
                {
                    ramp.mySound._volume = ramp.initialVolume *
                    (elapsedSinceStart / ramp.rampTime);
                }
                else
                {
                    ramp.mySound._volume = ramp.initialVolume;
                }

                if(ramp.mySound._volume >= ramp.initialVolume) {
                    ramp.mySound._volume = ramp.initialVolume;
                    allRamps.RemoveAt(i);
                    i--;
                }
            } else {

                if (ramp.rampTime > 0)
                {
                    ramp.mySound._volume = ramp.initialVolume *
                        (1.0f - (elapsedSinceStart / ramp.rampTime));
                }
                else
                {
                    ramp.mySound._volume = 0;
                }

                if(ramp.mySound._volume <= 0) {
                    ramp.mySound.Stop();
                    allRamps.RemoveAt(i);
                    ramp.mySound._volume = ramp.initialVolume;
                    i--;
                }
            }            
        }
    }


    //finds a sound with a specific name
    public Sound findSound(string name) {
        if (sounds_indexed.ContainsKey(name))
        { return sounds_indexed[name]; }
        return null;
    }

    public bool SoundIsPlaying(string name)
    {
        foreach (Ramp r in allRamps)
        {
            if (r.mySound.name == name)
            {
                return r.up;
            }
        }

        Sound thesound = findSound(name);
        if( thesound == null)
        {
            return false;
        }
        return thesound.isPlaying;
    }

    // If sound was marked as started and never stopped, then return it
    public bool IsSoundStarted(string name)
    {
        Sound thesound = findSound(name);
        if (thesound == null)
        {
            return false;
        }
        return thesound.started;
    }
}
