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
        ros.RegisterPublisher<HapticInfoMsg>("haptic_info");

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

    private void PublishHapticInfo(Int32Msg msg)
    {
        int buttonNumber = msg.data;

        Vector3 colliderPosition = collider.transform.position;
        Quaternion colliderRotation = collider.transform.rotation;

        TimeMsg rostime = new TimeMsg
        {
            sec = (int)Time.time,
            nanosec = (uint)((Time.time % 1) * 1e9)
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

        HapticInfoMsg hapticInfo = new HapticInfoMsg
        {
            button = buttonNumber,
            pose = colliderPoseStamped
        };

        ros.Publish("haptic_position", colliderPoseStamped);
        
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
            PublishHapticInfo(new Int32Msg(1));
        }

        if (buttons[1] == 1 && !button2Pressed)
        {
            button2Pressed = true;
            PublishHapticInfo(new Int32Msg(2));
        }
        else if (buttons[1] == 0 && button2Pressed)
        {
            button2Pressed = false;
        }

        if (buttons[0] == 0 && buttons[1] == 0)
            {
                PublishHapticInfo(new Int32Msg(0));
            }
    }
}