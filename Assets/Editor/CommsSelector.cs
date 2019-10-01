using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO.Ports;

[CustomEditor(typeof(MotionController))]
public class CommsSelector : Editor
{
    private string choice;

    string[] _availablePorts;

    private int _choiceIndex = 0;

    void OnEnable()
    {
        _availablePorts = SerialPort.GetPortNames();
    }

    public override void OnInspectorGUI()
    {
        // Draw the default inspector
        DrawDefaultInspector();

        EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

        if (Application.isPlaying)
        {
            var someClass = target as MotionController;
            bool isConnected = someClass.IsConnected();

            GUI.enabled = !isConnected;
            if (GUILayout.Button("Scan For Ports"))
            {
                _availablePorts = SerialPort.GetPortNames();
            }

            // If the choice is not in the array then the _choiceIndex will be -1 so set back to 0
            if (_choiceIndex < 0)
                _choiceIndex = 0;

            GUILayout.BeginHorizontal();
            if (_availablePorts != null)
                _choiceIndex = EditorGUILayout.Popup(_choiceIndex, _availablePorts);

            GUI.enabled = true;
            GUIStyle style = new GUIStyle();
            style.fontStyle = FontStyle.Bold;
            style.alignment = TextAnchor.LowerCenter;

            if (isConnected)
            {
                style.normal.textColor = new Color(0, 0.5f, 0);
                GUILayout.Label("Connected", style);
            }
            else
            {
                style.normal.textColor = new Color(0.5f, 0, 0);
                GUILayout.Label("Disconnected", style);
            }
            GUILayout.EndHorizontal();



            GUILayout.BeginHorizontal();

            GUI.enabled = !isConnected;
            if (GUILayout.Button("Connect"))
            {
                someClass.CreateAndOpenPort(_availablePorts[_choiceIndex]);
            }

            GUI.enabled = isConnected;
            if (GUILayout.Button("Disconnect"))
            {
                someClass.ClosePort();
            }

            GUI.enabled = true;
            GUILayout.EndHorizontal();

            // Save the changes back to the object
            EditorUtility.SetDirty(target);
        }
        else
        {
            GUIStyle style = new GUIStyle();
            style.fontStyle = FontStyle.Bold;
            style.alignment = TextAnchor.LowerCenter;
            GUILayout.Label("COMM options available when in run mode", style);
        }
    }
    //----------------------------------------------
}
//----------------------------------------------