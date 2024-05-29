using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class PauseButton : MonoBehaviour
{
    private Button button;
    public TcpManager networkManager;

    void Start()
    {
        // 获取Button组件
        button = GetComponent<Button>();
        // 添加点击事件处理器
        button.onClick.AddListener(OnButtonClick);
    }

    void OnButtonClick()
    {
        networkManager.SendCommand("kill");
    }
}