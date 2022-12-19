using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public  class CleanRunCopyToClip : MonoBehaviour
{
    public  Scorekeeper scorer = null;

    public  void CopyToClip()
    {
        if( scorer == null || !scorer.gameObject.activeInHierarchy )
        {
            scorer = GameObject.FindObjectOfType<Scorekeeper>();

            if( !scorer ) { return; }
        }

        scorer.CopyCleanCodeToSystemBuffer();
    }

}
