/* 
    ------------------- Code Monkey -------------------

    Thank you for downloading this package
    I hope you find it useful in your projects
    If you have any questions let me know
    Cheers!

               unitycodemonkey.com
    --------------------------------------------------
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using CodeMonkey.Utils;

public class Graph : MonoBehaviour {

    private static Graph instance;

    [SerializeField] private Sprite dotSprite;
    private RectTransform graphContainer;
    private RectTransform labelTemplateX;
    private RectTransform labelTemplateY;
    private RectTransform dashContainer;
    private RectTransform dashTemplateX;
    private RectTransform dashTemplateY;
    private List<GameObject> gameObjectList;
    private List<IGraphVisualObject> graphVisualObjectList;
    private GameObject tooltipGameObject;

    // Cached values
    private List<float> xvalueList;
    private List<float> yvalueList;
    private IGraphVisual graphVisual;
    private float xmax;
    private float ymax;

    private void Awake() {
        instance = this;
        // Grab base objects references
        graphContainer = transform.Find("graphContainer").GetComponent<RectTransform>();
        labelTemplateX = graphContainer.Find("labelTemplateX").GetComponent<RectTransform>();
        labelTemplateY = graphContainer.Find("labelTemplateY").GetComponent<RectTransform>();
        dashContainer = graphContainer.Find("dashContainer").GetComponent<RectTransform>();
        dashTemplateX = dashContainer.Find("dashTemplateX").GetComponent<RectTransform>();
        dashTemplateY = dashContainer.Find("dashTemplateY").GetComponent<RectTransform>();
        tooltipGameObject = graphContainer.Find("tooltip").gameObject;


        gameObjectList = new List<GameObject>();
        graphVisualObjectList = new List<IGraphVisualObject>();

        graphVisual = new LineGraphVisual(graphContainer, dotSprite, Color.green, new Color(1, 1, 1, .5f));

        // Set up base values

        yvalueList = new List<float>() { 0f, 0f, 0.02f, 0.08f, 0.25f, 0.57f, 1.08f, 1.83f, 2.84f, 4.14f, 5.73f, 7.64f, 9.85f, 12.4f, 15.2f, 18.4f };
        xvalueList = new List<float>() { 0f, 0.2f, 0.4f, 0.6f, 0.8f, 1f, 1.2f, 1.4f, 1.6f, 1.8f, 2f, 2.2f, 2.4f, 2.6f, 2.8f, 3f };
        xmax = 3f;
        ymax = 12f;

        HideTooltip();
    }

    public static void ShowTooltip_Static(string tooltipText, Vector2 anchoredPosition) {
        instance.ShowTooltip(tooltipText, anchoredPosition);
    }

    private void ShowTooltip(string tooltipText, Vector2 anchoredPosition) {
        // Show Tooltip GameObject
        tooltipGameObject.SetActive(true);

        tooltipGameObject.GetComponent<RectTransform>().anchoredPosition = anchoredPosition;

        Text tooltipUIText = tooltipGameObject.transform.Find("text").GetComponent<Text>();
        tooltipUIText.text = tooltipText;

        float textPaddingSize = 4f;
        Vector2 backgroundSize = new Vector2(
            tooltipUIText.preferredWidth + textPaddingSize * 2f, 
            tooltipUIText.preferredHeight + textPaddingSize * 2f
        );

        tooltipGameObject.transform.Find("background").GetComponent<RectTransform>().sizeDelta = backgroundSize;

        // UI Visibility Sorting based on Hierarchy, SetAsLastSibling in order to show up on top
        tooltipGameObject.transform.SetAsLastSibling();
    }

    public static void HideTooltip_Static() {
        instance.HideTooltip();
    }

    private void HideTooltip() {
        tooltipGameObject.SetActive(false);
    }

    public void UpdateGraph(List<float> xvalueList, List<float> yvalueList, float xmax, float ymax)
    {
        this.xvalueList = xvalueList;
        this.yvalueList = yvalueList;
        this.xmax = xmax;
        this.ymax = ymax;
        ShowGraph();
    }
 
    private void ShowGraph( ) {

        // Make sure initialization occured
        if( gameObjectList==null ) { return; }

        // Clean up previous graph
        foreach (GameObject gameObject in gameObjectList) {
            Destroy(gameObject);
        }
        gameObjectList.Clear();

        foreach (IGraphVisualObject graphVisualObject in graphVisualObjectList) {
            graphVisualObject.CleanUp();
        }
        graphVisualObjectList.Clear();

        graphVisual.CleanUp();

        // Grab the width and height from the container
        float graphWidth = graphContainer.rect.width;
        float graphHeight = graphContainer.rect.height;

        // Identify y Min and Max values
        float yMaximum = ymax;
        float yMinimum = 0f;

        float yDifference = yMaximum - yMinimum;
        if (yDifference <= 0) {
            yDifference = 0.1f;
        }

        // yMaximum = yMaximum + (yDifference * 0.2f);

        float xMaximum = xmax;
        float xMinimum = 0f;

        float xDifference = yMaximum - yMinimum;
        if (xDifference <= 0)
        {
            xDifference = 0.1f;
        }

        // xMaximum = xMaximum + (xDifference * 0.2f);
        float xSize = graphWidth / (xmax+ 1);

        // Cycle through all visible data points
        int xIndex = 0;
        bool out_of_bounds = false;

        for (int i = 0; i < xvalueList.Count; i++) {
            float currx = xvalueList[i];
            float curry = yvalueList[i];

            // If we are out of bounds, end loop
            if( out_of_bounds )
            {
                break;
            }

            // If we are below minimum (beggining of graph), get next point
            if (currx < 0f ||
                curry < 0f )
            {
                continue;
            }

            // Check for y value too high
            if (curry > ymax )
            {
                // If too high right from the beggining, get next point
                if( i==0 ) { continue; }

                // Drop all following points
                out_of_bounds = true;

                // Clip this point to the max
                Vector2 intersection = new Vector2(0, 0);
                if( ! LineIntersection( new Vector2( xMinimum - 1E+3f, ymax), 
                                        new Vector2( xMaximum + 1E+3f, ymax),
                                        new Vector2( xvalueList[i-1], yvalueList[i-1]),
                                        new Vector2(currx, curry),
                                        ref intersection) )
                { break;  }

                currx = intersection.x;
                curry = intersection.y;
            }

            // Check for xvalue too high
            if (currx > xmax)
            {
                // If too high right from the beggining, get next point
                if (i == 0) { continue; }

                // Drop all following points
                out_of_bounds = true;

                // Clip this point to the max
                Vector2 intersection = new Vector2(0, 0);
                if (!LineIntersection(new Vector2(xmax, yMinimum - 1E+3f),
                                        new Vector2(xmax, yMaximum + 1E+3f),
                                        new Vector2(xvalueList[i - 1], yvalueList[i - 1]),
                                        new Vector2(currx, curry),
                                        ref intersection))
                { break; }

                currx = intersection.x;
                curry = intersection.y;
            }

            float xPosition = ((currx - xMinimum) / (xMaximum - xMinimum)) * graphWidth;
            float yPosition = ((curry- yMinimum) / (yMaximum - yMinimum)) * graphHeight;

            // Add data point visual
            string tooltipText = "(" + currx.ToString("0.#") + "," + curry.ToString("0.#") + ")";
            IGraphVisualObject graphVisualObject = graphVisual.CreateGraphVisualObject(new Vector2(xPosition, yPosition), xSize, tooltipText);
            graphVisualObjectList.Add(graphVisualObject);

            xIndex++;
        }

        // Set up separators on the y axis
        float xseparatorCount = 6.001f;
        for (int i = 0; i <= xseparatorCount; i++)
        {
            // Duplicate the label template

            RectTransform labelX = Instantiate(labelTemplateX);
            labelX.SetParent(graphContainer, false);
            labelX.gameObject.SetActive(true);
            float normalizedValue = i * 1f / xseparatorCount;
            labelX.anchoredPosition = new Vector2(normalizedValue * graphWidth, -7f);
            labelX.GetComponent<Text>().text = (i * xmax / xseparatorCount).ToString("0.#");
            gameObjectList.Add(labelX.gameObject);

            // Duplicate the dash template
            RectTransform dashX = Instantiate(dashTemplateX);
            dashX.SetParent(dashContainer, false);
            dashX.gameObject.SetActive(true);
            dashX.anchoredPosition = new Vector2(normalizedValue * graphWidth, -3f);
            gameObjectList.Add(dashX.gameObject);
        }




        // Set up separators on the y axis
        float yseparatorCount = 12.001f;
        for (int i = 0; i <= yseparatorCount; i++) {
            // Duplicate the label template
           
            RectTransform labelY = Instantiate(labelTemplateY);
            labelY.SetParent(graphContainer, false);
            labelY.gameObject.SetActive(true);
            float normalizedValue = i * 1f / yseparatorCount;
            labelY.anchoredPosition = new Vector2(-7f, normalizedValue * graphHeight);
            labelY.GetComponent<Text>().text = (i * ymax / yseparatorCount).ToString("0.#");
            gameObjectList.Add(labelY.gameObject);

            // Duplicate the dash template
            RectTransform dashY = Instantiate(dashTemplateY);
            dashY.SetParent(dashContainer, false);
            dashY.gameObject.SetActive(true);
            dashY.anchoredPosition = new Vector2(-4f, normalizedValue * graphHeight);
            gameObjectList.Add(dashY.gameObject);      
        }
    }

    /*
     * Interface definition for showing visual for a data point
     * */
    private interface IGraphVisual {

        IGraphVisualObject CreateGraphVisualObject(Vector2 graphPosition, float graphPositionWidth, string tooltipText);
        void CleanUp();

    }

    /*
     * Represents a single Visual Object in the graph
     * */
    private interface IGraphVisualObject {

        void SetGraphVisualObjectInfo(Vector2 graphPosition, float graphPositionWidth, string tooltipText);
        void CleanUp();

    }


    /*
     * Displays data points as a Line Graph
     * */
    private class LineGraphVisual : IGraphVisual {

        private RectTransform graphContainer;
        private Sprite dotSprite;
        private LineGraphVisualObject lastLineGraphVisualObject;
        private Color dotColor;
        private Color dotConnectionColor;

        public LineGraphVisual(RectTransform graphContainer, Sprite dotSprite, Color dotColor, Color dotConnectionColor) {
            this.graphContainer = graphContainer;
            this.dotSprite = dotSprite;
            this.dotColor = dotColor;
            this.dotConnectionColor = dotConnectionColor;
            lastLineGraphVisualObject = null;
        }

        public void CleanUp() {
            lastLineGraphVisualObject = null;
        }


        public IGraphVisualObject CreateGraphVisualObject(Vector2 graphPosition, float graphPositionWidth, string tooltipText) {
            GameObject dotGameObject = CreateDot(graphPosition);


            GameObject dotConnectionGameObject = null;
            if (lastLineGraphVisualObject != null) {
                dotConnectionGameObject = CreateDotConnection(lastLineGraphVisualObject.GetGraphPosition(), dotGameObject.GetComponent<RectTransform>().anchoredPosition);
            }
            
            LineGraphVisualObject lineGraphVisualObject = new LineGraphVisualObject(dotGameObject, dotConnectionGameObject, lastLineGraphVisualObject);
            lineGraphVisualObject.SetGraphVisualObjectInfo(graphPosition, graphPositionWidth, tooltipText);
            
            lastLineGraphVisualObject = lineGraphVisualObject;

            return lineGraphVisualObject;
        }

        private GameObject CreateDot(Vector2 anchoredPosition) {
            GameObject gameObject = new GameObject("dot", typeof(Image));
            gameObject.transform.SetParent(graphContainer, false);
            gameObject.GetComponent<Image>().sprite = dotSprite;
            gameObject.GetComponent<Image>().color = dotColor;
            RectTransform rectTransform = gameObject.GetComponent<RectTransform>();
            rectTransform.anchoredPosition = anchoredPosition;
            rectTransform.sizeDelta = new Vector2(8, 8);
            rectTransform.anchorMin = new Vector2(0, 0);
            rectTransform.anchorMax = new Vector2(0, 0);
            
            // Add Button_UI Component which captures UI Mouse Events
            Button_UI dotButtonUI = gameObject.AddComponent<Button_UI>();

            return gameObject;
        }

        private GameObject CreateDotConnection(Vector2 dotPositionA, Vector2 dotPositionB) {
            GameObject gameObject = new GameObject("dotConnection", typeof(Image));
            gameObject.transform.SetParent(graphContainer, false);
            gameObject.GetComponent<Image>().color = dotConnectionColor;
            gameObject.GetComponent<Image>().raycastTarget = false;
            RectTransform rectTransform = gameObject.GetComponent<RectTransform>();
            Vector2 dir = (dotPositionB - dotPositionA).normalized;
            float distance = Vector2.Distance(dotPositionA, dotPositionB);
            rectTransform.anchorMin = new Vector2(0, 0);
            rectTransform.anchorMax = new Vector2(0, 0);
            rectTransform.sizeDelta = new Vector2(distance, 1f);
            rectTransform.anchoredPosition = dotPositionA + dir * distance * .5f;
            rectTransform.localEulerAngles = new Vector3(0, 0, UtilsClass.GetAngleFromVectorFloat(dir));
            return gameObject;
        }


        public class LineGraphVisualObject : IGraphVisualObject {

            public event EventHandler OnChangedGraphVisualObjectInfo;

            private GameObject dotGameObject;
            private GameObject dotConnectionGameObject;
            private LineGraphVisualObject lastVisualObject;

            public LineGraphVisualObject(GameObject dotGameObject, GameObject dotConnectionGameObject, LineGraphVisualObject lastVisualObject) {
                this.dotGameObject = dotGameObject;
                this.dotConnectionGameObject = dotConnectionGameObject;
                this.lastVisualObject = lastVisualObject;

                if (lastVisualObject != null) {
                    lastVisualObject.OnChangedGraphVisualObjectInfo += LastVisualObject_OnChangedGraphVisualObjectInfo;
                }
            }

            private void LastVisualObject_OnChangedGraphVisualObjectInfo(object sender, EventArgs e) {
                UpdateDotConnection();
            }

            public void SetGraphVisualObjectInfo(Vector2 graphPosition, float graphPositionWidth, string tooltipText) {
                RectTransform rectTransform = dotGameObject.GetComponent<RectTransform>();
                rectTransform.anchoredPosition = graphPosition;

                UpdateDotConnection();

                Button_UI dotButtonUI = dotGameObject.GetComponent<Button_UI>();

                // Show Tooltip on Mouse Over
                dotButtonUI.MouseOverOnceFunc = () => {
                    ShowTooltip_Static(tooltipText, graphPosition);
                };
            
                // Hide Tooltip on Mouse Out
                dotButtonUI.MouseOutOnceFunc = () => {
                    HideTooltip_Static();
                };

                if (OnChangedGraphVisualObjectInfo != null) OnChangedGraphVisualObjectInfo(this, EventArgs.Empty);
            }

            public void CleanUp() {
                Destroy(dotGameObject);
                Destroy(dotConnectionGameObject);
            }

            public Vector2 GetGraphPosition() {
                RectTransform rectTransform = dotGameObject.GetComponent<RectTransform>();
                return rectTransform.anchoredPosition;
            }

            private void UpdateDotConnection() {
                if (dotConnectionGameObject != null) {
                    RectTransform dotConnectionRectTransform = dotConnectionGameObject.GetComponent<RectTransform>();
                    Vector2 dir = (lastVisualObject.GetGraphPosition() - GetGraphPosition()).normalized;
                    float distance = Vector2.Distance(GetGraphPosition(), lastVisualObject.GetGraphPosition());
                    dotConnectionRectTransform.sizeDelta = new Vector2(distance, 3f);
                    dotConnectionRectTransform.anchoredPosition = GetGraphPosition() + dir * distance * .5f;
                    dotConnectionRectTransform.localEulerAngles = new Vector3(0, 0, UtilsClass.GetAngleFromVectorFloat(dir));
                }
            }

        }

    }

    // Copy from internet
    public static bool LineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 intersection)
    {
        float Ax, Bx, Cx, Ay, By, Cy, d, e, f, num;
        float x1lo, x1hi, y1lo, y1hi;

        Ax = p2.x - p1.x;
        Bx = p3.x - p4.x;

        // X bound box test/
        if (Ax < 0)
        {
            x1lo = p2.x; x1hi = p1.x;
        }
        else
        {
            x1hi = p2.x; x1lo = p1.x;
        }

        if (Bx > 0)
        {
            if (x1hi < p4.x || p3.x < x1lo) return false;
        }
        else
        {
            if (x1hi < p3.x || p4.x < x1lo) return false;
        }

        Ay = p2.y - p1.y;
        By = p3.y - p4.y;

        // Y bound box test//
        if (Ay < 0)
        {
            y1lo = p2.y; y1hi = p1.y;
        }
        else
        {
            y1hi = p2.y; y1lo = p1.y;
        }

        if (By > 0)
        {
            if (y1hi < p4.y || p3.y < y1lo) return false;
        }
        else
        {
            if (y1hi < p3.y || p4.y < y1lo) return false;
        }

        Cx = p1.x - p3.x;
        Cy = p1.y - p3.y;
        d = By * Cx - Bx * Cy;  // alpha numerator//
        f = Ay * Bx - Ax * By;  // both denominator//

        // alpha tests//
        if (f > 0)
        {
            if (d < 0 || d > f) return false;
        }
        else
        {
            if (d > 0 || d < f) return false;
        }

        e = Ax * Cy - Ay * Cx;  // beta numerator//

        // beta tests //
        if (f > 0)
        {
            if (e < 0 || e > f) return false;
        }
        else
        {
            if (e > 0 || e < f) return false;
        }

        // check if they are parallel
        if (f == 0) return false;

        // compute intersection coordinates //
        num = d * Ax; // numerator //
        intersection.x = p1.x + num  / f;

        num = d * Ay;
        intersection.y = p1.y + num / f;

        return true;
    }


}
