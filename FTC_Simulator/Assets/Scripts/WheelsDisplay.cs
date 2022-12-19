using System;
using UnityEngine;

namespace FTC_Simulator.Assets.Scripts
{

    public class WheelsDisplay : RobotSkin
    {
        // 4-wheel drive train
        public Transform WheelTL;
        public Transform WheelTR;
        public Transform WheelBL;
        public Transform WheelBR;

        // 6 wheel tank drive
        public Transform TWheelTL;
        public Transform TWheelTR;
        public Transform TWheelBL;
        public Transform TWheelBR;
        public Transform TWheelML;
        public Transform TWheelMR;

        private string my_DriveTrain = "";

        // heading of the body last update
        float lastBodyAngleRad;
        // location of the body last update
        Vector3 lastBodyPosition;

        override public void InitSkin()
        {
            base.InitSkin();

            // Find all the wheels
            WheelTL = MyUtils.FindHierarchy(transform, "MECWheelTL");
            WheelTR = MyUtils.FindHierarchy(transform, "MECWheelTR");
            WheelBL = MyUtils.FindHierarchy(transform, "MECWheelBL");
            WheelBR = MyUtils.FindHierarchy(transform, "MECWheelBR");

            TWheelTL = MyUtils.FindHierarchy(transform, "TANKWheelTL");
            TWheelTR = MyUtils.FindHierarchy(transform, "TANKWheelTR");
            TWheelBL = MyUtils.FindHierarchy(transform, "TANKWheelBL");
            TWheelBR = MyUtils.FindHierarchy(transform, "TANKWheelBR");
            TWheelML = MyUtils.FindHierarchy(transform, "TANKWheelML");
            TWheelMR = MyUtils.FindHierarchy(transform, "TANKWheelMR");

            // Assume this will be a 4-wheel drive system and thus disable 6-wheels and enable 4-wheels
            UpdateDriveTrain("Tank");
          
        }

        private void UpdateDriveTrain(string new_DriveTrain)
        {
            // If no change, than exit
            if( new_DriveTrain == my_DriveTrain) { return; }

            // Set the correct visuals
            my_DriveTrain = new_DriveTrain;

            switch (new_DriveTrain)
            {
                case "6-Wheel Tank":
                    if (WheelBL) { WheelBL.gameObject.SetActive(false); }
                    if (WheelBR) { WheelBR.gameObject.SetActive(false); }
                    if (WheelTL) { WheelTL.gameObject.SetActive(false); }
                    if (WheelTR) { WheelTR.gameObject.SetActive(false); }


                    if (TWheelBL) { TWheelBL.gameObject.SetActive(true); }
                    else          { WheelBL.gameObject.SetActive(true);  }
                    if (TWheelBR) { TWheelBR.gameObject.SetActive(true); }
                    else { WheelBR.gameObject.SetActive(true); }
                    if (TWheelTL) { TWheelTL.gameObject.SetActive(true); }
                    else { WheelTL.gameObject.SetActive(true); }
                    if (TWheelTR) { TWheelTR.gameObject.SetActive(true); }
                    else { WheelTR.gameObject.SetActive(true); }
                    if (TWheelML) { TWheelML.gameObject.SetActive(true); }
                    if (TWheelMR) { TWheelMR.gameObject.SetActive(true); }
                    break;

                default:
                    if (WheelBL) { WheelBL.gameObject.SetActive(true); }
                    if (WheelBR) { WheelBR.gameObject.SetActive(true); }
                    if (WheelTL) { WheelTL.gameObject.SetActive(true); }
                    if (WheelTR) { WheelTR.gameObject.SetActive(true); }


                    if (TWheelBL) { TWheelBL.gameObject.SetActive(false); }
                    if (TWheelBR) { TWheelBR.gameObject.SetActive(false); }
                    if (TWheelTL) { TWheelTL.gameObject.SetActive(false); }
                    if (TWheelTR) { TWheelTR.gameObject.SetActive(false); }
                    if (TWheelML) { TWheelML.gameObject.SetActive(false); }
                    if (TWheelMR) { TWheelMR.gameObject.SetActive(false); }
                    break;
            }
        }

        /// <summary>
        /// Call this every update to move the mechanum wheel visuals
        /// </summary>

        public float deltaTL;
        public float deltaTR;
        public float deltaBL;
        public float deltaBR;

        public void Update()
        {          
            if( !ri3d.rb_body || !WheelBL ||!WheelBR || !WheelTL || !WheelTR) { return; }
            UpdateDriveTrain(ri3d.DriveTrain);


            // Calculate how much the body turned and moved
            Vector3 currentBodyPos = ri3d.rb_body.transform.position;
            float currentBodyAngleRad = - AngleWrapDeg(ri3d.rb_body.transform.rotation.eulerAngles.y) * Mathf.Deg2Rad;

            currDeltaBodyRotationRad = AngleWrapRad(currentBodyAngleRad-lastBodyAngleRad);
            float bodyDistanceTraveledThisUpdate = Vector3.Distance(currentBodyPos,lastBodyPosition);

            // Calculate the change in the body's pos over this udpate
            float bodyDeltaX = currentBodyPos.x - lastBodyPosition.x;
            float bodyDeltaZ = currentBodyPos.z - lastBodyPosition.z;

            // What direction the body is actually moving
            float currentBodyMovementAbsoluteAngleRad = (float) Math.Atan2(bodyDeltaZ, bodyDeltaX);

            // What direction the body is moving relative to where it is pointing
            float relativeMovementAngle = currentBodyMovementAbsoluteAngleRad - currentBodyAngleRad;
            

            currDeltaBodyForwards = (float) Math.Cos(relativeMovementAngle) * bodyDistanceTraveledThisUpdate;
            currDeltaBodyStraif = (float) Math.Sin(relativeMovementAngle) * bodyDistanceTraveledThisUpdate;

            float forwardsAmount = currDeltaBodyForwards * 280;
            float straifAmount = currDeltaBodyStraif * 200;
            float turning_amount = currDeltaBodyRotationRad * 120;   

            // For tank drive, we don't care about straifing (this may be due to slippage?)
            if( my_DriveTrain == "6-Wheel Tank"  )
            {
                straifAmount = 0f;
            }

            deltaTL = forwardsAmount - straifAmount - turning_amount;
            deltaTR = forwardsAmount + straifAmount + turning_amount;
            deltaBL = forwardsAmount + straifAmount - turning_amount;
            deltaBR = forwardsAmount - straifAmount + turning_amount;


            if (WheelTL) 
            { 
                WheelTL.transform.Rotate(new Vector3(0, 1, 0), deltaTL); 
            }
            if (WheelTR) 
            { 
                WheelTR.transform.Rotate(new Vector3(0, 1, 0), deltaTR); 
            }
            if (WheelBL)
            {
                WheelBL.transform.Rotate(new Vector3(0, 1, 0), deltaBL);
            }
            if (WheelBR)
            {
                WheelBR.transform.Rotate(new Vector3(0, 1, 0), deltaBR);
            } 

            if (TWheelTL)
            {
                TWheelTL.transform.Rotate(new Vector3(0, 1, 0), deltaTL);
            }
            if (TWheelTR)
            {
                TWheelTR.transform.Rotate(new Vector3(0, 1, 0), deltaTR);
            }
            if (TWheelBL)
            {
                TWheelBL.transform.Rotate(new Vector3(0, 1, 0), deltaBL);
            }
            if (TWheelBR)
            {
                TWheelBR.transform.Rotate(new Vector3(0, 1, 0), deltaBR);
            }

            if (TWheelML) 
            { 
                TWheelML.transform.Rotate(new Vector3(0, 1, 0), deltaBL); 
            }
            if (TWheelMR)
            {
                TWheelMR.transform.Rotate(new Vector3(0, 1, 0), deltaBR);
            } 

            // save the current angle and position for calculating deltas next update
            lastBodyAngleRad = currentBodyAngleRad;
            lastBodyPosition = currentBodyPos;
        }

        public float currDeltaBodyForwards {get; private set; }
        public float currDeltaBodyStraif {get; private set; }
        public float currDeltaBodyRotationRad {get; private set; }

        /// The current speed of the robot in it's local forwards
        public float currentForwardsSpeed => currDeltaBodyForwards / Time.deltaTime;
        public float currentStraifSpeed => currDeltaBodyStraif / Time.deltaTime;
        public float currentRotationSpeedRad => currDeltaBodyRotationRad / Time.deltaTime;


        public static float AngleWrapDeg(float degrees) {
            float ret = degrees % 360;
            if(ret > 180) {
                ret -= 360;
            }
            return ret;
        }

        public static float AngleWrapRad(float rad) {
            float ret = rad % (2*Mathf.PI);
            if(ret > Mathf.PI) {
                ret -= 2 * Mathf.PI;
            }
            return ret;
        }


        Quaternion GetRelativeToBody(Transform myTransform) {
            return Quaternion.FromToRotation(myTransform.transform.right, ri3d.rb_body.transform.forward); 
        }
    }
}