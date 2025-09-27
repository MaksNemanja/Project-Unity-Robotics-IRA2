using UnityEngine;
using System.Runtime.InteropServices;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.UnityRoboticsIra2;

public class HapticButtonReader : MonoBehaviour
{
    ROSConnection ros;
    public GameObject collider;
    private int button_pressed;
    private float publishMessageFrequency = 0.008f;
    private float timeElapsed;
    
    // Importer la fonction getButtons depuis HapticsDirect
    [DllImport("HapticsDirect")]
    public static extern void getButtons(string configName, int[] buttons4, int[] last_buttons4, ref int inkwell);

    public HapticPlugin hapticPlugin;

    private int[] buttons = new int[4] { 0, 0, 0, 0 }; // État actuel des boutons
    private int[] lastButtons = new int[4] { 0, 0, 0, 0 }; // État précédent des boutons
    private int inkwell = 0; // État du commutateur inkwell

    private bool button2Pressed = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>("haptic_position");
        ros.RegisterPublisher<Int32Msg>("button_pressed");

        // Vérifier que la référence à HapticPlugin est définie
        if (hapticPlugin == null)
        {
            hapticPlugin = GetComponent<HapticPlugin>();
            if (hapticPlugin == null)
            {
                Debug.LogError("HapticButtonReader : Aucun composant HapticPlugin trouvé. Veuillez assigner une référence dans l'inspecteur.");
            }
        }
    }

    private void PublishPosition()
    {
        Vector3 colliderPosition = collider.transform.position;
        Quaternion colliderRotation = collider.transform.rotation;
        var now = System.DateTime.UtcNow;

        TimeMsg rostime = new TimeMsg
        {
            sec = (int)(now.Subtract(new System.DateTime(1970,1,1))).TotalSeconds,
            nanosec = (uint)((now.Ticks % System.TimeSpan.TicksPerSecond) * 100)
        };

        PoseStampedMsg colliderPoseStamped = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = rostime,
                frame_id = "world"
            },
            pose = new PoseMsg(
                new Vector3(colliderPosition.x, colliderPosition.y, colliderPosition.z).To<FLU>(),
                new Quaternion(colliderRotation.x, colliderRotation.y, colliderRotation.z, colliderRotation.w).To<FLU>()
            )
        };

        ros.Publish("haptic_position", colliderPoseStamped);
        
    }

    private void ButtonPressed(Int32Msg msg)
    {
        button_pressed = msg.data;
        ros.Publish("button_pressed", new Int32Msg(button_pressed));
        Debug.Log($"HapticButtonReader : Bouton {button_pressed} pressé.");
    }
        
    private void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            if (hapticPlugin != null && hapticPlugin.DeviceHHD >= 0)
            {
                UpdateButtonStatus();
            }
            timeElapsed = 0;
        }
    }

    private void UpdateButtonStatus()
    {
        // Sauvegarder l'état précédent des boutons
        for (int i = 0; i < 4; i++)
        {
            lastButtons[i] = buttons[i];
        }

        // Appeler getButtons pour mettre à jour l'état des boutons
        getButtons(hapticPlugin.DeviceIdentifier, buttons, lastButtons, ref inkwell);

        // Traiter les événements des boutons
        if (buttons[0] == 1)
        {
            ButtonPressed(new Int32Msg(1));
            PublishPosition();
        }

        if (buttons[1] == 1 && !button2Pressed)
        {
            button2Pressed = true;
            ButtonPressed(new Int32Msg(2));
        }
        else if (buttons[1] == 0 && button2Pressed)
        {
            button2Pressed = false;
        }

        if (buttons[0] == 0 && buttons[1] == 0)
            {
                ros.Publish("button_pressed", new Int32Msg(0));
                Debug.Log("HapticButtonReader : Aucun bouton pressé.");
            }
    }
}