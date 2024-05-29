using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class TcpManager : MonoBehaviour
{
    public GameObject TcpPanel;
    private TcpClient client;
    private NetworkStream stream;
    private TextMeshProUGUI ipInputField;
    private TextMeshProUGUI portInputField;
    private float connectInterval = 1f; // 连接间隔时间
    private float connectTimer = 0f; // 连接定时器

    void Start()
    {
        // 获取TCP IP和TCP Port的输入字段
        ipInputField = TcpPanel.transform.Find("IP/InputField/Text Area/Text").GetComponent<TextMeshProUGUI>();
        portInputField = TcpPanel.transform.Find("Port/InputField/Text Area/Text").GetComponent<TextMeshProUGUI>();

        if (ipInputField == null || portInputField == null)
        {
            Debug.LogError("Failed to find input fields.");
        }
    }

    void Update()
    {
        // 定时调用 Connect 方法
        connectTimer += Time.deltaTime;
        if (connectTimer >= connectInterval)
        {
            connectTimer = 0f;
            if (client == null || !client.Connected || stream == null)
            {
                Connect();
            }
        }
    }

    public void Connect()
    {
        // 获取用户输入的IP和端口
        string ip = ipInputField.text;
        ip = ip.Substring(0, ip.Length - 1);

        string s_port = portInputField.text;
        int port = 0;
        s_port = s_port.Substring(0, s_port.Length - 1);
        int.TryParse(s_port, out port);

        Debug.Log("Connecting to server: " + ip + ":" + port);

        // 创建TCP客户端
        client = new TcpClient();
        try
        {
            client.Connect(ip, port);
            Debug.Log("Connected to server.");
            // 获取网络流
            stream = client.GetStream();
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to connect to server: " + e.Message);
        }
    }

    public void SendCommand(string command)
    {
        if (client != null && client.Connected)
        {
            byte[] data = Encoding.UTF8.GetBytes(command);
            stream.Write(data, 0, data.Length);
            stream.Flush();
        }
        else
        {
            Debug.LogError("Not connected to server.");
        }
    }

    void OnDestroy()
    {
        if (client != null)
        {
            client.Close();
        }
    }
}