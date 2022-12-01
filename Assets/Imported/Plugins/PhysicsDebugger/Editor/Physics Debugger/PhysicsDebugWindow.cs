using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class PhysicsDebugWindow : EditorWindow {

	string selectedObjName = "";

	bool graphVelocity = true;
	bool graphAcceleration = true;
	bool graphVelocityAxis = false;
	bool graphAccelerationAxis = false;
	bool useVelocityColorLarge = true;
	bool useVelocityColorSmall = false;

	bool collisionStats = false;
	bool otherStats = false;

	float slowSpeed = 0f;
	float fastSpeed = 10f;

	Color slowSpeedColor = new Color(75f/255, 255f/255, 75f/255);
	Color fastSpeedColor = new Color(255f/255, 75f/255, 75f/255);

	int graphResolution = 250;
	int updateRate = 55;
	int DataRate = 55;

	PhysDebugGraphObj selectedGraphObj;
	List<PhysDebugGraphObj> pinnedGraphObjects = new List<PhysDebugGraphObj>();

	private static GUISkin editorSkin = null;


	[MenuItem ("Window/Physics Debugger Graphs")]
	public static void ShowWindow(){
		//Show existing window instance. If one doesn't exist, make one.
		EditorWindow.GetWindow(typeof(PhysicsDebugWindow));
		if(editorSkin == null)
			editorSkin = (GUISkin)(AssetDatabase.LoadAssetAtPath("Assets/Editor/Physics Debugger/PhysicsDebugWindowSkin.guiskin", typeof(GUISkin)));
	}
		
	public void OnInspectorUpdate(){
		if(!selectedGraphObj)
			getSelectedObj();
		List<PhysDebugGraphObj> sceneGraphObjs = Object.FindObjectsOfType<PhysDebugGraphObj>().ToList();
		sceneGraphObjs.RemoveAll(go => pinnedGraphObjects.Contains(go));
		sceneGraphObjs.Remove(selectedGraphObj);

		foreach(PhysDebugGraphObj pdgo in sceneGraphObjs){
			pdgo.Initialize();
		}

		pinnedGraphObjects.RemoveAll(pgo => pgo == null);

		pinnedGraphObjects.AddRange(sceneGraphObjs);

		if(selectedGraphObj){
			selectedGraphObj.graphResolution = graphResolution;
			selectedGraphObj.dataRate = DataRate;
		}

		pinnedGraphObjects = pinnedGraphObjects.Select(pgo => {pgo.graphResolution = graphResolution; return pgo;}).ToList();			
		pinnedGraphObjects = pinnedGraphObjects.Select(pgo => {pgo.dataRate = DataRate; return pgo;}).ToList();

	}

	int frameCounter = 0;
	bool onPlay = true;
	public void Update(){
		if(EditorApplication.isPlaying){
			if(!onPlay){
				onPlay = true;
				pinnedGraphObjects.Clear();
				DestroyImmediate(selectedGraphObj);
				getSelectedObj();
				for(int i = 0; i < pinnedGraphObjects.Count; i++){
					//pinnedGraphObjects[i] = new PhysDebugGraphObj(pinnedGraphObjects[i].gO);
				}
			}

			frameCounter++;
			if(frameCounter % (61 - updateRate) == 0){
				frameCounter = 0;
				if(selectedGraphObj == null)
					getSelectedObj();
				Repaint();
			}


		}else{
			if(onPlay){
				onPlay = false;
				for(int i = 0; i < pinnedGraphObjects.Count; i++){
					//pinnedGraphObjects[i] = new PhysDebugGraphObj(pinnedGraphObjects[i].gO);
				}
				selectedGraphObj = null;
				getSelectedObj();
				Repaint();
			}
		}
	}



	Vector2 scrollPos;
	void OnGUI(){
		pinnedGraphObjects.RemoveAll(pgo => pgo == null);

		if(editorSkin == null)
			editorSkin = (GUISkin)(AssetDatabase.LoadAssetAtPath("Assets/Editor/Physics Debugger/PhysicsDebugWindowSkin.guiskin", typeof(GUISkin)));		

		EditorGUILayout.BeginHorizontal();
		EditorGUILayout.PrefixLabel ("Selected Object: ", EditorStyles.boldLabel, EditorStyles.boldLabel);
		EditorGUILayout.LabelField (selectedObjName);
		EditorGUILayout.EndHorizontal();


		GUILayout.Space(15f);
		EditorGUILayout.LabelField ("Graph Options", EditorStyles.boldLabel);

		graphResolution = EditorGUILayout.IntSlider (new GUIContent("Graph Resolution",
			"Number of ticks that can fit on one graph."), graphResolution, 10, 2500);
		
		updateRate = EditorGUILayout.IntSlider (new GUIContent("Update Rate",
			"Rate at which the graphs are re-drawn. With high resolutions a fast refresh rate may cause slow-down."), updateRate, 1, 60);

		DataRate = EditorGUILayout.IntSlider (new GUIContent("Data Collection Rate",
			"Rate at which data is collected. 60 = every Physics update, 59 = every second Physics update, etc..."), DataRate, 1, 60);

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(10f));
		graphVelocity = EditorGUILayout.Toggle (new GUIContent("Velocity Magnitude",
			"If true, display velocity graphs for objects."), graphVelocity);
		GUILayout.Space(15f);
		graphAcceleration = EditorGUILayout.Toggle (new GUIContent("Acceleration Magnitude",
			"If true, display acceleration graphs for objects."), graphAcceleration);
		GUILayout.Space(15f);
		useVelocityColorLarge = EditorGUILayout.Toggle (new GUIContent("Color Acc/Vel by Speed",
			"If true, color the velocity/acceleration graphs by speed of objects. This creates a color gradient for the lines and is" 
			+ "very helpful for seeing how fast something is moving at a glance."), useVelocityColorLarge);
		EditorGUILayout.EndHorizontal();

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(10f));
		graphVelocityAxis = EditorGUILayout.Toggle (new GUIContent("Velocity Axes",
			"If true, display velocity graphs for the 3 world axes of objects."), graphVelocityAxis);
		GUILayout.Space(15f);
		graphAccelerationAxis = EditorGUILayout.Toggle (new GUIContent("Acceleration Axes",
			"If true, display acceleration graphs for the 3 world axes of objects."), graphAccelerationAxis);
		GUILayout.Space(15f);
		useVelocityColorSmall = EditorGUILayout.Toggle (new GUIContent("Color Axes by Speed",
			"If true, color the velocity/acceleration axes graphs by speed of objects. This creates a color gradient for the lines and is"
			+ "very helpful for seeing how fast something is moving at a glance."),useVelocityColorSmall);
		EditorGUILayout.EndHorizontal();

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(10f));
		collisionStats = EditorGUILayout.Toggle (new GUIContent("Collision Stats",
			"If true, display collision statistics for objects."), collisionStats);
		GUILayout.Space(15f);
		otherStats = EditorGUILayout.Toggle (new GUIContent("Velocity/Acc Stats",
			"If true, display various physics statistics for objects."), otherStats);
		EditorGUILayout.EndHorizontal();

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(10f));
		slowSpeed = EditorGUILayout.FloatField (new GUIContent("Low Speed Threshold",
			"The speed at which to start the slowSpeedColor for coloring the gradient lines when using the Velocity Color options."), slowSpeed);
		GUILayout.Space(15f);
		slowSpeedColor = EditorGUILayout.ColorField (new GUIContent("Low Speed Color",
			"The color to start with when coloring lines by velocity."), slowSpeedColor);
		EditorGUILayout.EndHorizontal();

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(10f));
		fastSpeed = EditorGUILayout.FloatField (new GUIContent("High Speed Threshold",
			"The speed at which to stop coloring the gradient lines when using the Velocity Color options."), fastSpeed);
		GUILayout.Space(15f);
		fastSpeedColor = EditorGUILayout.ColorField (new GUIContent("High Speed Color",
			"The color to end with when coloring lines by velocity."), fastSpeedColor);
		EditorGUILayout.EndHorizontal();

		//draw splitter
		GUILayout.Box("", new GUILayoutOption[]{GUILayout.ExpandWidth(true), GUILayout.Height(1)});

		scrollPos = GUILayout.BeginScrollView(scrollPos);

		if(selectedGraphObj != null){
			if(selectedGraphObj.gO){
				if(selectedGraphObj.rb){
					EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(15f));
					GUILayout.Label(selectedGraphObj.gO.name);
					if(GUILayout.Button("Pin") && pinnedGraphObjects.Find(x => x.gO == selectedGraphObj.gO) == null){
						pinnedGraphObjects.Add(selectedGraphObj);
						selectedGraphObj = null;
						selectedObjName = "None";
						Repaint();
						return;
					}
					EditorGUILayout.EndHorizontal();
					drawGraphsForObj(selectedGraphObj);
					if(collisionStats)
						drawCollisionStatsForObj(selectedGraphObj);
					if(otherStats){
						GUILayout.Space(10f);
						drawOtherStatsForObj(selectedGraphObj);
					}
				}else
					GUILayout.Label("Selected object has no rigid body.");
			}else
				GUILayout.Label("No selected rigid body.");
		}else
			GUILayout.Label("No selected rigid body.");

		foreach(PhysDebugGraphObj pGO in pinnedGraphObjects){
			GUILayout.Space(15f);
			EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(15f));
			GUILayout.Label(pGO.name);
			if(GUILayout.Button("Un-Pin")){
				pinnedGraphObjects.Remove(pGO);
				DestroyImmediate(pGO);
				Repaint();
				return;
			}
			if(!pGO.rb){
				GUILayout.Label("Object is static and will only display collision stats.");
				EditorGUILayout.EndHorizontal();
				drawCollisionStatsForObj(pGO);
			}else{
				EditorGUILayout.EndHorizontal();
				drawGraphsForObj(pGO);
				if(collisionStats)
					drawCollisionStatsForObj(pGO);
				if(otherStats){
					GUILayout.Space(10f);
					drawOtherStatsForObj(pGO);
				}
			}				
		}


		GUILayout.EndScrollView();
	}

	public Color GetVelocicolor(float inVel){
		float tPct = VelocityPercentage(inVel);
		return GradientPercent(slowSpeedColor, fastSpeedColor, tPct);
	}

	public float VelocityPercentage(float force){
		float dist = fastSpeed - slowSpeed;
		float forceAdjust = force - fastSpeed;
		return Mathf.Clamp01(forceAdjust / dist);
	}

	public Color GradientPercent(Color lower, Color upper, float percentage){
		return new Color(lower.r + percentage * (upper.r - lower.r),
			lower.g + percentage * (upper.g - lower.g),
			lower.b + percentage * (upper.b - lower.b));		
	}

	void OnSelectionChange(){
		getSelectedObj();
	}

	void getSelectedObj(){
		if(selectedGraphObj){
			DestroyImmediate(selectedGraphObj);
			selectedGraphObj = null;
			selectedObjName = "None";
		}

		if(Selection.activeGameObject && Selection.activeGameObject.activeInHierarchy){
			GameObject tObj = Selection.activeGameObject;
			if(tObj.GetComponent<PhysDebugGraphObj>()){
				selectedObjName = tObj.name;
				selectedObjName += ", Already Pinned";
				return;
			}
			selectedObjName = tObj.name;
			if(tObj.GetComponent<Rigidbody>()){
				selectedObjName += ", Has RigidBody";

				if(selectedGraphObj)
					DestroyImmediate(selectedGraphObj);
				selectedGraphObj = tObj.AddComponent<PhysDebugGraphObj>();
				selectedGraphObj.Initialize();
			}else
				selectedObjName += ", No RigidBody";

			Repaint();
		}
	}

	void drawCollisionStatsForObj(PhysDebugGraphObj gObj){
		GUI.enabled = false;
		GUI.color = new Color(1,1,1,2);
		Handles.BeginGUI();

		float boxWidth = 525;
		GUILayout.Space(5f);
		Rect lastRect = EditorGUILayout.BeginHorizontal();
		EditorGUILayout.EndHorizontal();
		GUILayout.Space(3f);
		Handles.color = Color.gray;
		Handles.DrawSolidRectangleWithOutline(new Rect(lastRect.x + 3, lastRect.yMax, boxWidth + 5, 152),
			new Color(0.9f ,0.9f ,0.9f), new Color(0.25f ,0.25f ,0.25f));


		float lastWidth = EditorGUIUtility.labelWidth;
		EditorGUILayout.BeginHorizontal(GUILayout.MinWidth(boxWidth));
		EditorGUIUtility.labelWidth = 60;
		EditorGUILayout.Toggle ("Colliding: ", gObj.colliding);
		GUILayout.Space(10f);
		EditorGUIUtility.labelWidth = 100;
		EditorGUILayout.IntField ("Collision Count: ", gObj.collisionCount);
		GUILayout.Space(05f);
		EditorGUIUtility.labelWidth = 125;
		EditorGUILayout.TextField ("Last Collided Object: ", gObj.collidingObject);
		EditorGUILayout.EndHorizontal();

		EditorGUIUtility.labelWidth = lastWidth;
		EditorGUILayout.BeginHorizontal(GUILayout.MinWidth(boxWidth));
		EditorGUILayout.TextField ("Last Collision Duration: ", gObj.collisionDuration.ToString("F2"));
		GUILayout.Space(10f);
		EditorGUILayout.TextField ("Time Without Collision: ", gObj.noCollisionDuration.ToString("F2"));
		EditorGUILayout.EndHorizontal();

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(boxWidth));
		EditorGUIUtility.labelWidth = 125;
		EditorGUILayout.FloatField ("Last Collision Force: ", gObj.collisionForce);
		GUILayout.Space(15f);
		EditorGUIUtility.labelWidth = 71;
		EditorGUILayout.FloatField ("Min Force: ", gObj.minCollisionForce);
		GUILayout.Space(15f);
		EditorGUILayout.FloatField ("Max Force: ", gObj.maxCollisionForce);
		EditorGUILayout.EndHorizontal();


		EditorGUILayout.Vector3Field("Last Relative Collision Velocity: ", gObj.collisionVelocity, GUILayout.MaxWidth(boxWidth));

		GUILayout.Space(5f);

		EditorGUILayout.BeginHorizontal(GUILayout.MinWidth(boxWidth));
		EditorGUIUtility.labelWidth = 68;
		EditorGUILayout.Toggle ("Triggering: ", gObj.triggering);
		GUILayout.Space(15f);
		EditorGUIUtility.labelWidth = 135;
		EditorGUILayout.TextField ("Last Triggering Object: ", gObj.triggeringObject);
		EditorGUILayout.EndHorizontal();
		
		EditorGUILayout.BeginHorizontal(GUILayout.MinWidth(boxWidth));
		EditorGUIUtility.labelWidth = 95;
		EditorGUILayout.IntField ("Trigger Count: ", gObj.triggerCount);
		GUILayout.Space(15f);
		EditorGUIUtility.labelWidth = 135;
		EditorGUILayout.TextField ("Last Trigger Duration: ", gObj.triggerDuration.ToString("F2"));
		EditorGUILayout.EndHorizontal();

		EditorGUIUtility.labelWidth = lastWidth;
		Handles.EndGUI();
		GUI.color = Color.white;
		GUI.enabled = true;
	}

	void drawOtherStatsForObj(PhysDebugGraphObj gObj){
		GUI.enabled = false;
		GUI.color = new Color(1,1,1,2);

		float boxWidth = 525;
		GUILayout.Space(5f);
		Rect lastRect = EditorGUILayout.BeginHorizontal();
		EditorGUILayout.EndHorizontal();
		GUILayout.Space(3f);
		Handles.color = Color.gray;
		Handles.DrawSolidRectangleWithOutline(new Rect(lastRect.x + 3, lastRect.yMax, boxWidth + 5, 165),
			new Color(0.9f ,0.9f ,0.9f), new Color(0.25f ,0.25f ,0.25f));

		float lastWidth = EditorGUIUtility.labelWidth;

		EditorGUILayout.BeginHorizontal(GUILayout.MaxWidth(boxWidth));
		EditorGUIUtility.labelWidth = 45;
		EditorGUILayout.FloatField ("Speed ", gObj.currentVelocity.magnitude);
		GUILayout.Space(15f);
		EditorGUIUtility.labelWidth = 72;
		EditorGUILayout.FloatField ("Min Speed: ", gObj.minSpeed);
		GUILayout.Space(15f);
		EditorGUILayout.FloatField ("Max Speed: ", gObj.maxSpeed);
		EditorGUILayout.EndHorizontal();

		EditorGUIUtility.labelWidth = lastWidth;

		EditorGUILayout.Vector3Field("Velocity: ", gObj.currentVelocity, GUILayout.MaxWidth(boxWidth));

		EditorGUILayout.Vector3Field("Min Velocity: ", gObj.minVelocity, GUILayout.MaxWidth(boxWidth));

		EditorGUILayout.Vector3Field("Max Velocity: ", gObj.maxVelocity, GUILayout.MaxWidth(boxWidth));

		EditorGUILayout.Vector3Field("Acceleration: ", gObj.currentAcceleration, GUILayout.MaxWidth(boxWidth));

		GUI.color = Color.white;
		GUI.enabled = true;
	}

	void drawGraphsForObj(PhysDebugGraphObj gObj){
		GUI.skin = editorSkin;
		float padding = 8f; float rectHeight = 125f; float rectWidth = position.width - padding * 2.75f;
		float halfWidth = rectWidth / 2f - padding / 2f;

		GUI.enabled = false;
		GUI.color = new Color(1,1,1,2);

		Color defRed = new Color(1f, 0.25f, 0.25f);
		Color defGreen = new Color(0.25f, 1f, 0.25f);
		Color defBlue = new Color(0.25f, 0.25f, 1f);

		//first set
		GUILayout.BeginHorizontal();
		if(graphVelocity){
			float twidth = rectWidth;
			if(graphAcceleration)
				twidth = halfWidth;
			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			Rect lastRect = GUILayoutUtility.GetLastRect();

			Handles.BeginGUI();
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Velocity Magnitude");
			drawVelocityGraph(lastRect, gObj.velocityGraph, defGreen, gObj, useVelocityColorLarge);
			Handles.EndGUI();
		}
		
		if(graphAcceleration){
			float twidth = rectWidth;
			if(graphVelocity)
				twidth = halfWidth;
			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			Rect lastRect = GUILayoutUtility.GetLastRect();
		
			Handles.BeginGUI( );
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Acceleration Magnitude");
			drawAccGraph(lastRect, gObj.accelerationGraph, defGreen, gObj, useVelocityColorLarge);
			Handles.EndGUI();
		}
		GUILayout.EndHorizontal();

		rectHeight *= 0.85f; rectWidth /= 3f; halfWidth /= 3f;
		halfWidth -= 3.3f; rectWidth -= 4.8f;
		if(graphVelocityAxis){
			float twidth = rectWidth;

			GUILayout.BeginHorizontal();
			Handles.BeginGUI();

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			Rect lastRect = GUILayoutUtility.GetLastRect();
			Handles.color = Color.white;
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Vel.X");
			drawAccGraph(lastRect, gObj.velocityAxisGraph.Select(x => -x.x).ToList(), defRed, null, useVelocityColorSmall);

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			lastRect = GUILayoutUtility.GetLastRect();
			Handles.color = Color.white;
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Vel.Y");
			drawAccGraph(lastRect, gObj.velocityAxisGraph.Select(x => -x.y).ToList(), defGreen, null, useVelocityColorSmall);

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			lastRect = GUILayoutUtility.GetLastRect();
			Handles.color = Color.white;
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Vel.Z");
			drawAccGraph(lastRect, gObj.velocityAxisGraph.Select(x => -x.z).ToList(), defBlue, null, useVelocityColorSmall);

			Handles.EndGUI();
			GUILayout.EndHorizontal();
		}

		if(graphAccelerationAxis){
			float twidth = rectWidth;

			GUILayout.BeginHorizontal();
			Handles.BeginGUI();

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			Rect lastRect = GUILayoutUtility.GetLastRect();
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Acc.X");
			drawAccGraph(lastRect, gObj.gGraph.Select(x => -x.x).ToList(), defRed, null, useVelocityColorSmall);

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			lastRect = GUILayoutUtility.GetLastRect();
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Acc.Y");
			drawAccGraph(lastRect, gObj.gGraph.Select(x => -x.y).ToList(), defGreen, null, useVelocityColorSmall);

			GUILayout.Button("", GUILayout.Width(twidth), GUILayout.Height(rectHeight));
			lastRect = GUILayoutUtility.GetLastRect();
			Handles.Label(new Vector3(lastRect.x + 5f, lastRect.y + 2f), "Acc.Z");
			drawAccGraph(lastRect, gObj.gGraph.Select(x => -x.z).ToList(), defBlue, null, useVelocityColorSmall);

			Handles.EndGUI();
			GUILayout.EndHorizontal();
		}


		GUI.enabled = true;
		GUI.skin = null;
		GUI.color = Color.white;
	}

	void drawVelocityGraph(Rect drawRect, List<float> graphList, Color drawColor, PhysDebugGraphObj inObj, bool useVelColor){
		if(graphList == null)
			return;
		Color defGreen = drawColor;
		Rect lastRect = drawRect;

		float step = lastRect.width / (float)graphResolution;
		int closest = -1;
		if(lastRect.Contains(Event.current.mousePosition) && EditorApplication.isPlaying){
			float mX = Event.current.mousePosition.x;
			Handles.color = new Color(1,1,1,0.4f);
			Handles.DrawLine(new Vector3(mX, drawRect.y), new Vector3(mX, drawRect.yMax));

			List<float> graphPoss = new List<float>();
			float tPoss = drawRect.xMax;
			for(int i = 0; i <= graphResolution; i++){
				graphPoss.Add(tPoss);
				tPoss -= step;
			}
			graphPoss.Reverse();

			float closestF = graphPoss.Aggregate((x,y) => Mathf.Abs(x-mX) < Mathf.Abs(y-mX) ? x : y);
			closest = graphPoss.IndexOf(closestF);
		}

		lastRect.height -= 1f;
		if(graphList.Count > 0){
			float max = Mathf.Clamp(graphList.Max() + 2.75f, 3f, 999999f);
			float lastVel = 1f - graphList[graphList.Count - 1] / max;
			Vector3 lastGPos = new Vector3(lastRect.xMax, lastVel * lastRect.height + lastRect.y);
			Vector3 gPos = new Vector3(lastRect.xMax, 0f);
			gPos.x -= step;

			GUI.skin.label.alignment = TextAnchor.UpperRight;
			Handles.Label(new Vector3(drawRect.xMax, drawRect.y + 4f), max.ToString("F1"));
			GUI.skin.label.alignment = TextAnchor.LowerRight;
			Handles.Label(new Vector3(drawRect.xMax, drawRect.yMax), "0");
			GUI.skin.label.alignment = TextAnchor.UpperLeft;

			bool drawPoint = false;
			Vector2 drawPointPos = Vector2.zero;
			for(int i = graphList.Count - 1; i >= 0; i--){
				float vel = graphList[i];
				float velPct = 1f - vel / max;
				gPos.y = velPct * lastRect.height + lastRect.y;
				if(i + (graphResolution - graphList.Count + 1) == closest){
					drawPoint = true;
					drawPointPos = gPos;
					if(drawRect.xMax - gPos.x < 80f){
						GUI.skin.label.alignment = TextAnchor.LowerRight;
						Handles.Label(gPos + new Vector3(-9, 3), vel.ToString("F2"));
					}else{
						GUI.skin.label.alignment = TextAnchor.LowerLeft;
						Handles.Label(gPos + new Vector3(9, 3), vel.ToString("F2"));
					}
					GUI.skin.label.alignment = TextAnchor.UpperLeft;
				}
				if(i == graphList.Count - 1)
					continue;
				if(i + 1 < graphList.Count)
					lastVel = graphList[i + 1];
				else
					lastVel = 0f;

				if(useVelColor)
					Handles.color = GetVelocicolor(Mathf.Abs(lastVel));
				else
					Handles.color = defGreen;
				Handles.DrawLine(lastGPos, gPos);

				lastGPos = gPos;
				gPos.x -= step;
			}
			if(drawPoint){
				Handles.color = new Color(1,1,1,0.4f);
				Handles.DrawSolidDisc(drawPointPos, Vector3.forward, 4.9f);
			}
		}
	}

	void drawAccGraph(Rect drawRect, List<float> graphList, Color drawColor, PhysDebugGraphObj inObj, bool useVelColor){
		Color defGreen = drawColor;
		Rect lastRect = drawRect;

		float step = lastRect.width / (float)graphResolution;
		int closest = -1;

		Handles.color = new Color(1,1,1,0.4f);
		float halfHeight = drawRect.y + drawRect.height / 2f;
		Handles.DrawLine(new Vector3(drawRect.x, halfHeight), new Vector3(drawRect.xMax, halfHeight));

		if(lastRect.Contains(Event.current.mousePosition) && EditorApplication.isPlaying){
			float mX = Event.current.mousePosition.x;
			Handles.color = new Color(1,1,1,0.4f);
			Handles.DrawLine(new Vector3(mX, drawRect.y), new Vector3(mX, drawRect.yMax));

			List<float> graphPoss = new List<float>();
			float tPoss = drawRect.xMax;
			for(int i = 0; i <= graphResolution; i++){
				graphPoss.Add(tPoss);
				tPoss -= step;
			}
			graphPoss.Reverse();

			float closestF = graphPoss.Aggregate((x,y) => Mathf.Abs(x-mX) < Mathf.Abs(y-mX) ? x : y);
			closest = graphPoss.IndexOf(closestF);
		}

		Handles.color = defGreen;
		lastRect.height -= 1f;
		if(graphList.Count > 0){
			float max = Mathf.Clamp(graphList.Max() + 1f, 2f, 999999f);
			float min = Mathf.Clamp(Mathf.Abs(graphList.Min()) + 1f, 2f, 999999f);
			if(min > max)
				max = min;
			else
				min = max;
			min = -min;
			float scaledMax = max - min;
			float lastVel = graphList[graphList.Count - 1] - min;
			lastVel = lastVel / scaledMax;
			Vector3 lastGPos = new Vector3(lastRect.xMax, lastVel * lastRect.height + lastRect.y);
			Vector3 gPos = new Vector3(lastRect.xMax, 0f);
			gPos.x -= step;

			GUI.skin.label.alignment = TextAnchor.UpperRight;
			Handles.Label(new Vector3(drawRect.xMax, drawRect.y + 4f), max.ToString("F1"));
			GUI.skin.label.alignment = TextAnchor.LowerRight;
			Handles.Label(new Vector3(drawRect.xMax, drawRect.yMax), min.ToString("F1"));
			GUI.skin.label.alignment = TextAnchor.UpperLeft;

			bool drawPoint = false;
			Vector2 drawPointPos = Vector2.zero;
			for(int i = graphList.Count - 1; i >= 0; i--){
				float acc = graphList[i];
				acc -= min;
				float velPct = acc / scaledMax;
				gPos.y = velPct * lastRect.height + lastRect.y;
				if(i + (graphResolution - graphList.Count + 1) == closest){
					drawPoint = true;
					drawPointPos = gPos;
					if(drawRect.xMax - gPos.x < 80f){
						GUI.skin.label.alignment = TextAnchor.LowerRight;
						Handles.Label(gPos + new Vector3(-9, 3), (-(acc + min)).ToString("F2"));
					}else{
						GUI.skin.label.alignment = TextAnchor.LowerLeft;
						Handles.Label(gPos + new Vector3(9, 3), (-(acc + min)).ToString("F2"));
					}
					GUI.skin.label.alignment = TextAnchor.UpperLeft;
				}
				if(i == graphList.Count - 1)
					continue;
				if(i + 1 < graphList.Count)
					lastVel = graphList[i + 1];
				else
					lastVel = 0f;
				if(useVelColor)
					Handles.color = GetVelocicolor(Mathf.Abs(lastVel));

				Handles.DrawLine(lastGPos, gPos);

				lastGPos = gPos;
				gPos.x -= step;
			}
			if(drawPoint){
				Handles.color = new Color(1,1,1,0.4f);
				Handles.DrawSolidDisc(drawPointPos, Vector3.forward, 4.9f);
			}
		}
	}

	//end window class
}