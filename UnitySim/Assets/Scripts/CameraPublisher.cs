using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera unityCamera;
    public string topicName = "unity/camera/image_raw";
    public int publishRate = 30; // Hz
    
    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timer;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        
        // Create render texture for camera
        renderTexture = new RenderTexture(640, 480, 24);
        unityCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }
    
    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f / publishRate)
        {
            PublishImage();
            timer = 0;
        }
    }
    
    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture2D.Apply();
        
        // Flip the image horizontally
        FlipTextureHorizontally(texture2D);
        
        ImageMsg message = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "unity_camera"
            },
            height = (uint)texture2D.height,
            width = (uint)texture2D.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(texture2D.width * 3),
            data = texture2D.GetRawTextureData()
        };
        
        ros.Publish(topicName, message);
    }
    
    void FlipTextureHorizontally(Texture2D texture)
    {
        Color[] pixels = texture.GetPixels();
        Color[] flippedPixels = new Color[pixels.Length];
        
        int width = texture.width;
        int height = texture.height;
        
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                flippedPixels[y * width + x] = pixels[y * width + (width - 1 - x)];
            }
        }
        
        texture.SetPixels(flippedPixels);
        texture.Apply();
    }
}