using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wire : MonoBehaviour
{
    public int weight = 9999;
    public JunctionStuff node1;
    public GenericFieldTracker terminal = null;
    public JunctionStuff node2;
    public MeshRenderer myrenderer;
    public Material RedMaterial;
    public Material BlueMaterial;
    public Material RedLightningMaterial;
    public Material BlueLightningMaterial;
    public bool show = true;
    public bool partofcircuit = false;
    public bool red_terminal = false;
    public bool blue_terminal = false;
    public bool red_owned = false;
    public bool blue_owned = false;
    public Scorekeeper scorekeeper;

    public Vector3 myscale;
    public particleAttractorLinear lightning_ps;
    private Color red_lightning = new Color(1f, 0.15f, 0.15f);
    private Color blue_lightning = new Color(0.15f, 0.15f, 1f);

    // Awake is called when this wire is first created.
    void Awake()
    {
        // Turn off renderer
        myrenderer.enabled = false;
        myscale = transform.localScale;
        myscale.x *= myrenderer.transform.localScale.x;
        myscale.y *= myrenderer.transform.localScale.y;
        myscale.z *= myrenderer.transform.localScale.z;

        // Get the 2 points that represent our outer edges
        Vector3 point1 = myrenderer.transform.position;
        point1 += myrenderer.transform.right * myscale.x * 10f / 2;
        Vector3 point2 = myrenderer.transform.position;
        point2 -= myrenderer.transform.right * myscale.x * 10f / 2;

        // Now find the junction that is closest to this point
        JunctionStuff[] alljunctions = FindObjectsOfType<JunctionStuff>();
        node1 = FindClosestJunction(alljunctions, point1);
        node2 = FindClosestJunction(alljunctions, point2);

        // Make node1 always have something there (in case this is the terminal end)
        if( node1==null)
        {
            node1 = node2;
        }

        // Create circuit connection
        node1.connected_wires.Add(this);

        if( terminal == null)
        {
            node2.connected_wires.Add(this);
        }

        // Init lightning
        lightning_ps.gameObject.SetActive(false);

        // Get the scorekeeper
        scorekeeper = FindObjectOfType<Scorekeeper>();
    }

    private JunctionStuff FindClosestJunction(JunctionStuff[] junctions, Vector3 point)
    {
        JunctionStuff closest = junctions[0];
        float distance = Vector3.Distance(closest.transform.position, point);

        foreach (JunctionStuff curr in junctions)
        {
            // Drop terminals
            if( curr.myType == JunctionStuff.JunctionType.terminal) { continue; }

            float newdistance = Vector3.Distance(curr.transform.position, point);
            if ( newdistance < distance )
            {
                distance = newdistance;
                closest = curr;
            }
        }

        // Make sure distance is really close
        if( distance > 1f) { return null; }

        return closest;
    }

    private void Update()
    {
        // Don't do anything in client mode
        if( GLOBALS.CLIENT_MODE) { return; }
       
        // If game isn't running, don't do anything
        if( scorekeeper && !scorekeeper.IsTimerRunning()) { return; }     

        red_owned = false;
        blue_owned = false;


        // Determine ownership
        // Case that this wire connects to a terminal
        if (red_terminal)
        {
            if ((terminal.GetGameElementCount(ElementType.Red1) > 0) && node1.red_owned)
            {
                red_owned = true;               
            }
        }
        else if (blue_terminal)
        {
            if ((terminal.GetGameElementCount(ElementType.Blue1) > 0) && node1.blue_owned)
            {                
                blue_owned = true;                
            }
        }
        else // Otherwise do regular wire check
        {
            if (node1.red_owned && node2.red_owned)
            {                
                red_owned = true;
            }
            else if (node1.blue_owned && node2.blue_owned)
            {              
                blue_owned = true;
            }
        }

        UpdateVisuals();
    }

    // Settting and reading states for internet connection
    public string GetState(bool nooffset = true)
    {
        // If we aren't showing, return 0
        if( !show || !red_owned && !blue_owned)
        { return "0"; }

        int return_code = 0;

        // Send color and lightning 
        if( red_owned)
        {
            return_code = (partofcircuit) ? 2 : 1;
        }
        else
        {
            return_code = (partofcircuit) ? 4 : 3;
        }
    
        // If an offset is requrest, add it in
        if( !nooffset )
        {
            return_code += 4;
        }

        return return_code.ToString();
    }

    public void SetState(char state)
    {
        // Set variables to default values 
        show = true;
        red_owned = false;
        blue_owned = false;
        partofcircuit = false;

        // Set correct state
        switch (state)
        {
            case '0':
                show = false;
                break;

            case '1':
                red_owned = true;
                break;

            case '2':
                red_owned = true;
                partofcircuit = true;
                break;

            case '3':
                blue_owned = true;
                break;

            case '4':
                blue_owned = true;
                partofcircuit = true;
                break;
        }

        // If we are a spectator, decode spectator states
        if (GLOBALS.topclient && GLOBALS.topclient.isSpectator())
        {
            switch (state)
            {             
                case '5':
                    red_owned = true;
                    break;

                case '6':
                    red_owned = true;
                    partofcircuit = true;
                    break;

                case '7':
                    blue_owned = true;
                    break;

                case '8':
                    blue_owned = true;
                    partofcircuit = true;
                    break;
            }
        }

        UpdateVisuals();
    }

    public void UpdateVisuals()
    {
        // Turn off visuals if nothing to show
        if( !show || !red_owned && !blue_owned)
        {
            // Turn off lightning
            if (lightning_ps.gameObject.activeSelf)
            {
                lightning_ps.gameObject.SetActive(false);
            }

            // Turn off renderer
            myrenderer.enabled = false;

            return;
        }

        // Turn on correct visuals
        Material newmaterial;
        
        if(blue_owned)
        {
            newmaterial = (partofcircuit) ? BlueLightningMaterial : BlueMaterial;
        }
        else 
        {
            newmaterial = (partofcircuit) ? RedLightningMaterial : RedMaterial;
        }

        if (myrenderer.sharedMaterial != newmaterial)
        {
            myrenderer.sharedMaterial = newmaterial;        
        }

        myrenderer.enabled = true;

        // Set lightning color
        if ( partofcircuit)
        {
            lightning_ps.SetTrailColor( (blue_owned) ? blue_lightning : red_lightning);
            
            // Turn on lightning
            if (!lightning_ps.gameObject.activeSelf)
            {
                lightning_ps.gameObject.SetActive(true);
            }
        }
        else
        {
            // Turn on lightning
            if (lightning_ps.gameObject.activeSelf)
            {
                lightning_ps.gameObject.SetActive(false);
            }
        }
    }
}
