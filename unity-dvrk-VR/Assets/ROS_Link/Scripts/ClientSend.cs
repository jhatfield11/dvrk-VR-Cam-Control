using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace ROSLink
{
    public class ClientSend : MonoBehaviour
    {
        private static void SendTCPData(Packet _packet)
        {
            _packet.WriteLength();
            Client.instance.tcp.SendData(_packet);
        }

        private static void SendUDPData(Packet _packet)
        {
            _packet.WriteLength();
            Client.instance.udp.SendData(_packet);
        }

        #region Packets
        public static void WelcomeReceived()
        {
            using (Packet _packet = new Packet((int)ClientPackets.welcomeReceived))
            {
                _packet.Write(Client.instance.myId);
                _packet.Write(UIManager.instance.nameField.text);

                SendTCPData(_packet);
                Debug.Log("Packet Sent");
            }
        }


        public static void ControllerAngles(List<string> _jointNames, List<float> _curAngs, string _botName)
        {
            using (Packet _packet = new Packet((int)ClientPackets.controllerAngles))
            {
                _packet.Write(_botName);
                _packet.Write(_jointNames.Count);
                for (int i = 0; i < _jointNames.Count; i++)
                {
                    _packet.Write(_jointNames[i]);
                    _packet.Write(_curAngs[i]);
                }
                SendUDPData(_packet);
            }
        }
        #endregion
    }
}