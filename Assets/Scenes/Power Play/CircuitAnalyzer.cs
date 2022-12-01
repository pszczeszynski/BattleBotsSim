using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CircuitAnalyzer : MonoBehaviour
{
    public List<Wire> red_start = new List<Wire>();
    public List<Wire> blue_start = new List<Wire>();

    public List<Wire> red_finish = new List<Wire>();
    public List<Wire> blue_finish = new List<Wire>();

    private List<JunctionStuff> all_junctions;
    private List<Wire> all_wires;

    public int blue_path = 9999;
    public int red_path = 9999;

    // Start is called before the first frame update
    void Start()
    {
        // Find all junctions
        all_junctions = new List<JunctionStuff>(FindObjectsOfType<JunctionStuff>());
        all_wires = new List<Wire>(FindObjectsOfType<Wire>());

        // Reset all junction weights
        ResetWeights();
    }


    void ResetWeights()
    {
        foreach(JunctionStuff curr in all_junctions)
        {
            if (curr.red_owned || curr.blue_owned) { curr.weight = 9999; }      
        }

        foreach (Wire curr in all_wires)
        {
            if (curr.red_owned || curr.blue_owned) { curr.weight = 9999; }
        }

        foreach (Wire curr in red_start ) { curr.weight = 1; }
        foreach (Wire curr in blue_start) { curr.weight = 1; }

    }

    // Fills the connections with the lowest weights
    // Recursive function
    void FindPath(JunctionStuff junction, int weight, bool isred )
    {
        // Make sure this is a step forward
        if(junction==null || junction.weight < weight ) { return; }

        // Record the weight in the junction
        junction.weight = weight;

        // Go through each off the junctions wires
        foreach(Wire curr in junction.connected_wires)
        {
            // If the wire is a valid hop
            if( (curr.red_owned && isred || curr.blue_owned && !isred) && (curr.weight > weight))
            {
                // Record the new weight for the wire
                curr.weight = weight;

                // Go to its junctions downstream
                FindPath(curr.node1, weight + 1, isred);
                FindPath(curr.node2, weight + 1, isred);

            }
        }
    }


    // Update is called once per frame
    void Update()
    {
        // Find all the weights
        ResetWeights();
        foreach(Wire curr in red_start)
        {
            if( curr.red_owned) { FindPath(curr.node1, 2, true); };
        }

        foreach (Wire curr in blue_start)
        {
            if (curr.blue_owned) { FindPath(curr.node1, 2, false); };
        }

        // See if we found a path
        foreach(Wire curr in red_finish)
        {
            if( curr.red_owned && curr.weight < red_path) { red_path = curr.weight; }
        }

        // See if we found a path
        foreach (Wire curr in blue_finish)
        {
            if (curr.blue_owned && curr.weight < blue_path) { blue_path = curr.weight; }
        }
    }
}
