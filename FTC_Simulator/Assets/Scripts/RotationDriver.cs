using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotationDriver : MonoBehaviour
{
    public enum TransformParameter
    {
        PositionX,
        PositionY,
        PositionZ,
        RotationX,
        RotationY,
        RotationZ,
    }



    // Can either be driven or be used as reference
    [Serializable]
    public class AnimationParameter {
        public string myTransformName;
        public Transform myTransform;

        public string myReferenceName;
        public Transform myReference;
        public TransformParameter myParameter;
        public bool isAnAngle => myParameter == TransformParameter.RotationX || 
                                myParameter == TransformParameter.RotationY ||
                                myParameter == TransformParameter.RotationZ;
        private bool initialized = false;

        public Vector3 initialPos;
        public Vector3 initialRot;
        public float lastValueAbs;
        
        // starts tracking the rotation
        public void Initialize(Transform topparent = null )
        {
            if( topparent != null)
            {
                if ((myTransform == null) && (myTransformName.Length > 0)) // Find our references
                {
                    myTransform = MyUtils.FindHierarchy(topparent, myTransformName);
                }

                if( (myReference == null) && (myReferenceName.Length > 0))
                {
                    myReference = MyUtils.FindHierarchy(topparent, myReferenceName);                  
                }
            }

            lastValueAbs = currentValueAbs;
            initialized = true;
        }

        public void SetInitialPos()
        {
            initialPos = myReference.worldToLocalMatrix.MultiplyPoint(myTransform.position);
            initialRot = (Quaternion.Inverse(myReference.rotation) * myTransform.rotation).eulerAngles;
            Initialize();
        }

        public float currentValue
        {
            get {
                return currentValueAbs;
                if (isAnAngle)
                {
                    float ret = lastValueAbs + AngleWrap(currentValueAbs - lastValueAbs);
                    lastValueAbs = ret;
                    return ret;
                }
                else
                {
                    return currentValueAbs;
                }
            }
            set
            {
                currentValueAbs = value;
            }
        }
        // The value of the target parameter (relative to the reference)
        // Ex if our paramter was positionx, then we return our position x relative to the reference
        public float currentValueAbs {
            get {
                if( !myReference || !myTransform ) { return 0;  }

                Vector3 lPos = myReference.worldToLocalMatrix.MultiplyPoint(myTransform.position);
                Vector3 lRot = (Quaternion.Inverse(myReference.rotation) * myTransform.rotation).eulerAngles;

                if (myParameter == TransformParameter.PositionX) return lPos.x;
                if (myParameter == TransformParameter.PositionY) return lPos.y;
                if (myParameter == TransformParameter.PositionZ) return lPos.z;

                if (myParameter == TransformParameter.RotationX) return lRot.x;
                if (myParameter == TransformParameter.RotationY) return lRot.y;
                if (myParameter == TransformParameter.RotationZ) return lRot.z;
                return -1f;
            }
            
            set {
                if (!myReference || !myTransform) { return; }

                // First, get our location and rotation relative to our reference, we will then modify them, then re-apply them
                Vector3 lPos = myReference.worldToLocalMatrix.MultiplyPoint(myTransform.position);
                Vector3 lRot = (Quaternion.Inverse(myReference.rotation) * myTransform.rotation).eulerAngles;

                //if (!initialized)
                //{
                //    Initialize();
                //}


                // modify the value that is used to drive the animation (since we want to set the driver back to that)
                lPos = initialPos;
                if (myParameter == TransformParameter.PositionX) { lPos.x = value; lPos.y = initialPos.y; lPos.z = initialPos.z; }
                if (myParameter == TransformParameter.PositionY) { lPos.x = initialPos.x; lPos.y = value; lPos.z = initialPos.z; }
                if (myParameter == TransformParameter.PositionZ) { lPos.x = initialPos.x; lPos.y = initialPos.y; lPos.z = value; }

                lRot = initialRot;
                if (myParameter == TransformParameter.RotationX) { lRot.x = value; lRot.y = initialRot.y; lRot.z = initialRot.z; }
                if (myParameter == TransformParameter.RotationY) { lRot.x = initialRot.x; lRot.y = value; lRot.z = initialRot.z; }
                if (myParameter == TransformParameter.RotationZ) { lRot.x = initialRot.x; lRot.y = initialRot.y; lRot.z = value; }
                
                // now convert the modified relative positions and rotations back to world space and apply
                myTransform.rotation = myReference.rotation * Quaternion.Euler(lRot);
                myTransform.position = myReference.localToWorldMatrix.MultiplyPoint(lPos);
            }
        }
    }

    /// <summary>
    /// Sets the initial position of all the transform parameters
    /// </summary>
    public void SetInitialPos()
    {
        myTargets.ForEach((target) => { target.SetInitialPos(); });
    }

    public AnimationParameter myDriver;
    public List<AnimationParameter> myTargets;


    /// <summary>
    /// Gets the current percent complete the driver is (0 when at the first keyframe, 1 at the last keyframe)
    /// </summary>
    float getPercentOnDriver(float value) {
        if (!myDriver.isAnAngle) {
            return (value - minimumDriverValue)/(maximumDriverValue - minimumDriverValue);
        } else {
            return (value - minimumDriverValue) / getTotalDriverRange();
        }
    }

    float getTotalDriverRange()
    {
        if (myDriver.isAnAngle)
        {
            float totalDistance = 0;
            for (int i = 1; i < myKeyFrames.Count; i++)
            {
                totalDistance += AngleWrap(myKeyFrames[i].driverValue - myKeyFrames[i - 1].driverValue);
            }
            return totalDistance;
        } else
        {
            return maximumDriverValue - minimumDriverValue;
        }
        
    }

    [Serializable]
    public class KeyFrame
    {
        public float driverValue;
        public List<float> targetValues;

        public KeyFrame() {
            targetValues = new List<float>();
        }
    }

    public List<KeyFrame> myKeyFrames;
    public float driver_value;
    public float driver_percentage;

    void Update()
    {
        // The index of the keyframe closest to the left of where our current value is
        int indexKeyFrameBefore = 0;
        driver_value = AngleWrap(myDriver.currentValue);

        float driverPercent = getPercentOnDriver(driver_value);
        driver_percentage = driverPercent;

        if (driverPercent > 0.999f) driverPercent = 0.999f;
        if (driverPercent < 0.0001f) driverPercent = 0.0001f;
        while (indexKeyFrameBefore < myKeyFrames.Count - 2 && getPercentOnDriver(myKeyFrames[indexKeyFrameBefore + 1].driverValue) < driverPercent) {
            indexKeyFrameBefore ++;
        }

        // the first keyframe to the left of us
        KeyFrame leftKeyFrame = myKeyFrames[indexKeyFrameBefore];
        // the first keyframe to the right of us
        KeyFrame rightKeyFrame = myKeyFrames[indexKeyFrameBefore + 1];

        // 0 when we are directly over the left keyframe, 1 when over the right keyframe
        float keyframeBlendPercent;
        if (!myDriver.isAnAngle) keyframeBlendPercent = (myDriver.currentValue - leftKeyFrame.driverValue) / (rightKeyFrame.driverValue - leftKeyFrame.driverValue);
        else keyframeBlendPercent = AngleWrap(myDriver.currentValue - leftKeyFrame.driverValue) / AngleWrap(rightKeyFrame.driverValue - leftKeyFrame.driverValue);

        if (keyframeBlendPercent > 0.999f) keyframeBlendPercent = 0.999f;
        if (keyframeBlendPercent < 0.0001f) keyframeBlendPercent = 0.0001f;

        for (int i = 0; i < myTargets.Count; i ++) {
            // APPLY TO TARGETS
            if (!myTargets[i].isAnAngle) {
                myTargets[i].currentValue = keyframeBlendPercent * (rightKeyFrame.targetValues[i] - leftKeyFrame.targetValues[i]) + leftKeyFrame.targetValues[i];
            } else {
                myTargets[i].currentValue = AngleWrap(keyframeBlendPercent * AngleWrap(rightKeyFrame.targetValues[i] - leftKeyFrame.targetValues[i]) + leftKeyFrame.targetValues[i]);
            }
        }
    }


    private void Start()
    {
        // Retrieve top-parent transform so sub-classes can find their links
        RobotInterface3D top_of_robot = GetComponentInParent<RobotInterface3D>();
        Transform top_transform = (top_of_robot == null) ? null : top_of_robot.transform;

        foreach (AnimationParameter p in myTargets)
        {
            p.Initialize(top_transform);
        }

        myDriver.Initialize(top_transform);
    }


    public Transform topparent_ineditor = null;
    public void InitializeInEditor()
    {
        foreach (AnimationParameter p in myTargets)
        {
            p.Initialize(topparent_ineditor);
        }

        myDriver.Initialize(topparent_ineditor);
    }


    /// Called from the editor when we want to add a keyframe
    public void AddKeyframe()
    {
        if (myKeyFrames == null)
        {
            myKeyFrames = new List<KeyFrame>();
        }
        KeyFrame newKeyFrame = new KeyFrame();
        newKeyFrame.driverValue = myDriver.currentValue;

        // add all the current values of the targets to the keyframe
        myTargets.ForEach((ap) => newKeyFrame.targetValues.Add(ap.currentValue));
        myKeyFrames.Add(newKeyFrame);
    }

    public void SetKeyframe() {
        KeyFrame editMe = myKeyFrames[EditingKeyframe];
        editMe.driverValue = myDriver.currentValue;

        for (int i = 0; i < myTargets.Count; i++) {
            editMe.targetValues[i] = myTargets[i].currentValue;
        }
    }

    /// The 100% of our animation
    private float minimumDriverValue => myKeyFrames[0].driverValue;
    /// The 0% of our animation
    private float maximumDriverValue => myKeyFrames[myKeyFrames.Count - 1].driverValue;


    /// Called from the editor
    public void RemoveKeyframe()
    {
        myKeyFrames.RemoveAt(myKeyFrames.Count - 1);
    }

    public static float AngleWrap(float angle) {
        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public float AngleWrapLight(float angle)
    {
        while (angle > 360)
        {
            angle -= 360;
        }

        while (angle < -360)
        {
            angle += 360;
        }
        return angle;
    }

    //////////////EDITOR VARIABLES/////////////////
    public int EditingKeyframe = 0;
    ///////////////////////////////////////////////

    public void RestoreKeyframe() 
    {
        for (int i = 0; i < myTargets.Count; i ++ ) {
            myTargets[i].currentValue = myKeyFrames[EditingKeyframe].targetValues[i];
        }
        myDriver.currentValue = myKeyFrames[EditingKeyframe].driverValue;
    }
}
