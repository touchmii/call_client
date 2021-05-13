###无线按钮盒
应用场景，AGV叫料器，远程设备启动开关。

支持通信协议： TCP、HTTP、WebSockets、MQTT、ModBusTCP
支持功能：可通过Web或HTTP接口配置按钮功能，使用某种协议，协议主机和端口等。
Wi-Fi未连接成功指示灯快闪提示（200ms），并开启AP模式提供Wi-Fi配置功能
上电长按按钮超过10秒钟恢复出厂设置，期间指示灯慢闪

改进：
改用单总线指示灯，添加一个OLED显示屏，增加一路按钮。

|--------------|-------|---------------|--|--|--|--|--|
^              ^       ^               ^     ^
Sketch    OTA update   File system   EEPROM  WiFi config (SDK)

4MB的储存可以分配1、2、3M为文件系统