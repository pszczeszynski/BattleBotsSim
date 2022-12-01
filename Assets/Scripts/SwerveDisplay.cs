using System;
using UnityEngine;

namespace FTC_Simulator.Assets.Scripts
{

    public class SwerveDisplay : RobotSkin
    {
        // 4-wheel drive train
        public Transform WheelTL;
        public Transform WheelTR;
        public Transform WheelBL;
        public Transform WheelBR;

        // location of the body last update
        Vector3 last_WheelTL;
        Vector3 last_WheelTR;
        Vector3 last_WheelBL;
        Vector3 last_WheelBR;

        public float TL_angle = 0;
        public float TR_angle = 0;
        public float BL_angle = 0;
        public float BR_angle = 0;


        override public void InitSkin()
        {
            base.InitSkin();

            // Find all the wheels
            if (WheelTL == null) { WheelTL = MyUtils.FindHierarchy(transform, "SWERVE_TL"); }
            if (WheelTR == null) { WheelTR = MyUtils.FindHierarchy(transform, "SWERVE_TR"); }
            if (WheelBL == null) { WheelBL = MyUtils.FindHierarchy(transform, "SWERVE_BL"); }
            if (WheelBR == null) { WheelBR = MyUtils.FindHierarchy(transform, "SWERVE_BR"); }
          
        }

        /// <summary>
        /// Call this every update to move the mechanum wheel visuals
        /// </summary>

        private float DoWheel( Transform wheel, ref Vector3 lastpos)
        {
            // Get the vector moved in real world space
            Vector3 delta_mov = wheel.transform.position - lastpos;

            // Project it onto the body's plane (y-vector is the normal to this plane = transform.up)
            // We simply subtract the portion of the vector that points up and are left with the xz plane only
            Vector3 delta_mov_xz = delta_mov - Vector3.Project(delta_mov, ri3d.rb_body.transform.up);

            // If the movement is too small, don't calculate
            if (delta_mov_xz.magnitude < 0.5E-3) { return wheel.localRotation.eulerAngles.y; }
            lastpos = wheel.transform.position;


            // Get the angle between the vectors. The difference between this and Vector3.angle is this doesn't just return the absolute shortest angle, but keeps the polarity
            // correct.
            float angle_change = -1f*MyUtils.AngleBetweenVectors(delta_mov_xz, ri3d.rb_body.transform.right, ri3d.rb_body.transform.up);

            Quaternion rot = wheel.localRotation;
            Vector3 euler = rot.eulerAngles;
            euler.y = angle_change;
            rot.eulerAngles = euler;
            wheel.localRotation = rot;

            return angle_change;
        }

        public void Update()
        {          
            if( !ri3d.rb_body || !WheelBL ||!WheelBR || !WheelTL || !WheelTR) { return; }

            TL_angle = DoWheel(WheelTL, ref last_WheelTL);
            TR_angle = DoWheel(WheelTR, ref last_WheelTR);
            BL_angle = DoWheel(WheelBL, ref last_WheelBL);
            BR_angle = DoWheel(WheelBR, ref last_WheelBR);        
        }
      
    }
}