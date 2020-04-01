#include "gps.h"

#define UBX_CONFIG_TIMEOUT	250		// ms, timeout for waiting ACK
#define GPS_READ_BUFFER_SIZE 150 ///< buffer size for the receive() call. Messages can be longer than that.


u16                               _ack_waiting_msg = 0;  //等待应答的命令class--id
u16                                        _rx_msg = 0;  //gps发回数据的类型
ubx_ack_state_t              _ack_state = UBX_ACK_IDLE;  //等待应答状态机
ubx_decode_state_t    _decode_state = UBX_DECODE_SYNC1;  //gps数据解析程序状态机
ubx_rxmsg_state_t	        _rx_state = UBX_RXMSG_IGNORE;  //MCU对接收的不同class数据消息处理方式状态机
u8                                        _rx_ck_a = 0;  //接收校验低字节
u8                                        _rx_ck_b = 0;  //接收校验高字节
u16                             _rx_payload_length = 0;  //解析gps一帧数据时有效数据的长度
u16                              _rx_payload_index = 0;  //gps一帧数据中当前被解析的有效数据索引(0 1 2 ...n)
u8                                      _configured =0;  //0-gps未配置，1--gps已配置
ubx_buf_t                                         _buf;  //gps数据暂存buffer
EXYF_FOLLOW                                follow_data;  //跟随数据


//gps配置
void gps_config(){

  ubx_payload_tx_cfg_prt_t    cfg_prt;
  ubx_payload_tx_cfg_rate_t   cfg_rate;
  ubx_payload_tx_cfg_nav5_t   cfg_nav5;
  ubx_payload_tx_cfg_msg_t    cfg_msg;
  //串口配置数据
  memset(&cfg_prt,0,sizeof(ubx_payload_tx_cfg_prt_t));
  cfg_prt.portID       = UBX_TX_CFG_PRT_PORTID;           //配置gps串口
  cfg_prt.mode         = UBX_TX_CFG_PRT_MODE;             //8位字长-无奇偶校验-1位停止位
  cfg_prt.baudRate     = UBX_TX_CFG_PRT_BAUDRATE;         //波特率115200
  cfg_prt.inProtoMask  = UBX_TX_CFG_PRT_INPROTOMASK_GPS;  //输入协议ubx
  cfg_prt.outProtoMask = UBX_TX_CFG_PRT_OUTPROTOMASK_GPS; //输出协议ubx
	
  while(1){
		
    sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)(&cfg_prt), sizeof(ubx_payload_tx_cfg_prt_t));//发送配置串口数据---注意这里有指针类型的强制转换
	
    if(waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT) > 0)//等待应答
       continue;
    else
       break;
		
  }
	
  //更新频率
  memset(&cfg_rate,0,sizeof(ubx_payload_tx_cfg_rate_t));
  cfg_rate.measRate = UBX_TX_CFG_RATE_MEASINTERVAL; //10hz
  cfg_rate.navRate  = UBX_TX_CFG_RATE_NAVRATE;
  cfg_rate.timeRef  = UBX_TX_CFG_RATE_TIMEREF;      //utc时间(格林尼治时间)
	
  while(1){
    sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(ubx_payload_tx_cfg_rate_t));
		
    if(waitForAck(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT) > 0)//等待应答
       continue;
    else
       break;
		
  }
	
  //gps内部滤波设置
  memset(&cfg_nav5,0,sizeof(ubx_payload_tx_cfg_nav5_t));
  cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
  cfg_nav5.dynModel	= UBX_TX_CFG_NAV5_DYNMODEL;
  cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;
	
  while(1){
    sendMessage(UBX_MSG_CFG_NAV5, (uint8_t *)&cfg_nav5, sizeof(ubx_payload_tx_cfg_nav5_t));
		
    if(waitForAck(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT) > 0)//等待应答
       continue;
    else
       break;
		
  }
	
  //设置PVT数据更新频率
  memset(&cfg_msg,0,sizeof(ubx_payload_tx_cfg_msg_t));
  cfg_msg.cmid_s2c1.cmid_combine = UBX_MSG_NAV_PVT;
  cfg_msg.port_uart1_rate = 1;      /* 该值与CFG RATE设置的测量频率紧密相关，PVT更新频率 = （CFG RATE/该值） 例如，当CFG RATE=10hz,1 means 10Hz, 2 means 5Hz */
	
  while(1){
    sendMessage(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
		
    if(waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT) > 0)//等待应答
       continue;
    else
       break;
		
  }
	
  _configured = 1;
	follow_data.head[0]    = '$';
	follow_data.head[1]    = 'E';
	follow_data.head[2]    = 'X';
	follow_data.head[3]    = 'Y';
	follow_data.head[4]    = 'F';
	follow_data.buflen     = 48;
	follow_data.command    = 20;
	follow_data.command_re = 20;
	follow_data.crc_check  = 0x3f00;
  LED0 = !LED0;  //提示gps配置完成	
}


//发送给gps
/*
  参数：msg--配置命令class-id
       *payload--有效数据指针
       length--有效数据长度
  返回值：无
*/
void sendMessage(const u16 msg, const u8 *payload, const u16 length){
	
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
	ubx_checksum_t checksum = {0, 0};
	
	const u8 *p;
	u16 i;
	// Populate header
	header.msg	= msg;
	header.length	= length;
	
	// Calculate checksum
	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes
	
	if (length > 0) {
		calcChecksum(payload, length, &checksum);
	}
	
	p = (uint8_t *)&header;
	for(i=0;i < sizeof(header);i++){
		   usart2_send(*(p+i));		
	}
	
	// Send message
	for(i=0;i < length;i++){
		  usart2_send(*(payload+i));
	}
	
	p = (uint8_t *)&checksum;
	for(i=0;i < sizeof(checksum);i++){
		  usart2_send(*(p+i));	
	}
	
}


//校验和计算
/*
  参数：*buffer--指向需要计算检验值的数据首址
        length--将计算校验和的数据长度
        *checksum--指向最终计算的校验和
  返回值：无
*/
void calcChecksum(const u8 *buffer, const u16 length, ubx_checksum_t *checksum){
	u16 i;
	for (i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}	
}


//配置后等待gps返回应答
/*
  参数：msg--等待应答的配置命令class-id
        timeout--等待时间
  返回值：0--ok 1--failed配置失败
*/
u8 waitForAck(const u16 msg, const u16 timeout){

    _ack_state = UBX_ACK_WAITING;
    _ack_waiting_msg = msg;
    decodeInit();
  
    receive(timeout);
	
    if(_ack_state == UBX_ACK_GOT_ACK){
       return 0;	// ACK received ok
    }else{
       _ack_state = UBX_ACK_IDLE;
       return 1; // ACK received failed
    }		
				
}
//接收gps数据
/*
参  数： timeout--解析等待时间
返回值： 0--成功 1--失败
*/
u8 receive(u16 timeout)
{
    u16 i;
    u8 handle = 0;
    u8 buf_tmp[GPS_READ_BUFFER_SIZE];
    time_sys_start(timeout);
	
    while(poll_time_sys_ms() > 0){
      u16 ret = usart2_read(GPS_READ_BUFFER_SIZE,buf_tmp);
      if(ret > 0){//接收到数据后解析数据
         for(i=0; i<ret; i++){
            handle|=parseChar(buf_tmp[i]);
         }	
      }
      if(handle > 0 )
         break;
    }
	
    if(handle > 0)
       handle =0;
    else 
       handle = 1;

    time_sys_stop();
    decodeInit();
    return handle;
}


//解析gps数据
/*
  参数：b--要解析的数据
  返回值：0--正在解析；1--信息被处理,解析完毕
*/
u8 parseChar(const u8 b){

    u8 ret = 0;
    switch (_decode_state) {
		
        /* Expecting Sync1 */
    case UBX_DECODE_SYNC1:
		
        if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
            _decode_state = UBX_DECODE_SYNC2;
        } 
        break;
		
		
    /* Expecting Sync2 */
    case UBX_DECODE_SYNC2:
        if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
            _decode_state = UBX_DECODE_CLASS;

        } else {		// Sync1 not followed by Sync2: reset parser
            decodeInit();
        }
        break;
		
		
    /* Expecting Class */
    case UBX_DECODE_CLASS:
        addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
        _rx_msg = b;
        _decode_state = UBX_DECODE_ID;
        break;
	
	
    /* Expecting ID */
    case UBX_DECODE_ID:
        addByteToChecksum(b);
        _rx_msg |= b << 8;
        _decode_state = UBX_DECODE_LENGTH1;
        break;
	
	
    /* Expecting first length byte */
    case UBX_DECODE_LENGTH1:
        addByteToChecksum(b);
        _rx_payload_length = b;
        _decode_state = UBX_DECODE_LENGTH2;
        break;
	
	
    /* Expecting second length byte */
    case UBX_DECODE_LENGTH2:
        addByteToChecksum(b);
        _rx_payload_length |= b << 8;	// calculate payload size

        if (payloadRxInit() != 0) {	// start payload reception
            // payload will not be handled, discard message
            decodeInit();

        } else {
            _decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
        }

        break;
		
		
    /* Expecting payload */
    case UBX_DECODE_PAYLOAD:
        addByteToChecksum(b);
	
        switch (_rx_msg) {

        default:
            ret = payloadRxAdd(b);		// add a payload byte
            break;
        }

        if (ret > 0) {
            // payload complete, expecting checksum
            _decode_state = UBX_DECODE_CHKSUM1;

        } 

        ret = 0;
        break;
	
	
    /* Expecting first checksum byte */
    case UBX_DECODE_CHKSUM1:
        if (_rx_ck_a != b) {
            decodeInit();

        } else {
            _decode_state = UBX_DECODE_CHKSUM2;
        }

        break;
	
	
    /* Expecting second checksum byte */
    case UBX_DECODE_CHKSUM2:
        if (_rx_ck_b != b) {
			

        } else {
            ret = payloadRxDone();	// finish payload processing
        }

        decodeInit();
        break;
	

    default:
        break;
    }
    return ret;
}

//解析初始化
void decodeInit(){
	
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
	_rx_msg = 0;
}


//gps接收数据的校验
/*
  参数：b--待计算校验数据
  返回值：无
*/
void addByteToChecksum(const u8 b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

//有效数据接收初始化
/*
  返回值： 0-继续解析  1--放弃本次解析
*/
u8 payloadRxInit(){
	
    u8 ret = 0;
	
    _rx_state = UBX_RXMSG_HANDLE;	// handle by default
	
   switch (_rx_msg) {
	
   case UBX_MSG_NAV_PVT:
        if (!_configured) {
            _rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

        } else if (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8) {	/* u-blox 8+ msg format */
            _rx_state = UBX_RXMSG_ERROR_LENGTH;

        } 
		
        break;
	
    case UBX_MSG_ACK_ACK:
        if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;

        } else if (_configured) {
            _rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
        }
		
		
        break;
		
		
		
		
		
    default:
        _rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
        break;
		
    }
	
    switch (_rx_state) {
	
    case UBX_RXMSG_HANDLE:	// handle message
    case UBX_RXMSG_IGNORE:	// ignore message 
          ret = 0;
          break;
		
    case UBX_RXMSG_DISABLE:	// disable unexpected messages
        ret = 1;	// return error, abort handling this message
        break;	
		
    case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
        ret = 1;	// return error, abort handling this message
        break;
		
    default:	// invalid message state
        ret = 1;	// return error, abort handling this message
        break;
    }
	
    return ret;
	
}

	
//有效数据添加到解析结构体中
/*
  b--要添加的数据
  返回值： 0 = ok 正常添加有效数据, 1 = 有效数据读取完毕
*/
u8 payloadRxAdd(const u8 b)
{
	int ret = 0;
	u8 *p_buf = (u8 *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}


//解析有效数据
/* 
  返回值： 0--解析失败，无对应解析数据 1--解析成功
*/
u8 payloadRxDone(){
	
    u8 ret = 0;
    int tmp; 
		
    // return if 无需要处理的信息
    if(_rx_state != UBX_RXMSG_HANDLE){
        return ret;
    }
	
    switch (_rx_msg){
		
		//解析PVT数据
    case UBX_MSG_NAV_PVT:
		
        tmp                   = _buf.payload_rx_nav_pvt.lat;
        follow_data.lat       = (float)tmp*1e-7f;
        tmp                   = _buf.payload_rx_nav_pvt.lon;
        follow_data.lon       = (float)tmp*1e-7f;
        tmp                   = _buf.payload_rx_nav_pvt.hMSL;
        follow_data.alt       = tmp * 1e-3f;
        follow_data.vy        =  (float)_buf.payload_rx_nav_pvt.velE * 1e-3f;
        follow_data.vx        =  (float)_buf.payload_rx_nav_pvt.velN * 1e-3f;
        follow_data.vz        =  (float)_buf.payload_rx_nav_pvt.velD * 1e-3f;
        follow_data.mag_dec   = _buf.payload_rx_nav_pvt.magDec;
        follow_data.flags     = _buf.payload_rx_nav_pvt.flags;
        follow_data.fixType   = _buf.payload_rx_nav_pvt.fixType;
        follow_data.numSV     = _buf.payload_rx_nav_pvt.numSV;	
		    
	
        ret = 1;
        break;
	
	  //配置成功
    case UBX_MSG_ACK_ACK:
        if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
            _ack_state = UBX_ACK_GOT_ACK;
        }

        ret = 1;
        break;
	
		//配置未成功
    case UBX_MSG_ACK_NAK:

        if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
           _ack_state = UBX_ACK_GOT_NAK;
        }

        ret = 1;
        break;

    default:
        break;
    }
	
    return ret;
}
























