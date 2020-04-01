gps配置：
   UBX_MSG_CFG_PRT--串口波特率115200 1-8-0-1 输入输出协议ubx
   等待应答
   UBX_MSG_CFG_RATE--收集数据频率 100ms、10hz 格林尼治时间
   等待应答
   UBX_MSG_CFG_NAV5--模式设置
   等待应答
   UBX_MSG_CFG_MSG--指定某类id数据在串口端更新频率
   等待应答
   
数据接收解析：
   只接收解析UBX_MSG_NAV_PVT数据