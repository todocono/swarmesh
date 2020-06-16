class Communication
{
private:
    AsyncUDP _udp;
    char *_ssid;
    char *_password;
    void _udp_init(Destination dstc);

public:
    Communication();
    void communication_init(Destination dstc);
};

Communication::Communication(char *ssid, char *password)
{
    _udp;
    _ssid = ssid;
    _password = password;
}

void Communication::communication_init()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("WiFi Failed");
        while (1)
        {
            delay(1000);
        }
    }
}