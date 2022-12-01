Thanks for using my physics debug script!


Setup:
This Package contains a couple different things for debugging purposes.
It has a PhysicsDebugger Component which can draw collision normals in world/local space and scale/color
them by impact force, as well as draw a trail from the object and color it by the objects velocity.
Similarly it can draw a trajectory prediction and color it by predicted velocity.
To use this, add the component to a GameObject with a rigidbody and set the settings and it's good to go.
It can be found under Physics/Physics Debugger.

Secondly it has a custom editor window used for logging graphs of an object's
velocity/acceleration and velocity/acceleration on all 3 axis.
It can be found under Window/Physics Debugger Graphs.
The options in the window can be changed to suit your needs and it will automatically graph velocity/acceleration
for the selected object, and objects can be pinned using the pin button so it will stay graphed when not selected.
To find an exact value for a spot on the graph, just hover the mouse over it.
If you add a PhysDebugGraphObj to prefab it will automatically be pinned to the graph when it is instantiated.


Hopefully all the settings have good tooltips and the script does everything you need.
If not, feel free to send me a message about issues or ideas on features to be added :]