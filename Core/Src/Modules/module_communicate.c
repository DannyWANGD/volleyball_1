 #include "module_communicate.h"
  #include "module_platform.h"
 #include "cmsis_os.h"
 #include "periph_remote.h"
 #include "lib_buff.h"
 #include "util_uart.h"
 #include "periph_motor.h"
 #include "util_usb.h"
 #include "sys_dwt.h"
 #include "sys_const.h"
 #include "app_remote.h"
 #include "app_gimbal.h"
#include <string.h>
 
static uint32_t _send_Remote_data(uint8_t *buff);   // send remote data
static uint32_t _send_Platform_data(uint8_t *buff); // send platform data
static uint32_t _send_Gimbal_data(uint8_t *buff);   // send gimbal data
static uint32_t _set_Platform_Data_(uint8_t *buff); // set platform data
static uint32_t _set_Gimbal_Data_(uint8_t *buff);   // set gimbal data

Protocol_DataTypeDef Protocol_Data;

/* 接收数据累积缓冲，用于处理 DMA 分片/拼包情况 */
#define PROTOCOL_RX_ACCUM_SIZE (Const_Com_RX_BUFF_LEN * 4)
static uint8_t rx_accum[PROTOCOL_RX_ACCUM_SIZE];
static volatile uint32_t rx_accum_len = 0;

Protocol_DataTypeDef* Protocol_GetBusDataPtr() {
    return &Protocol_Data;
}

// Reset protocol data timing
void Protocol_ResetProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}

void Protocol_InitProtocol() {

    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Protocol_ResetProtocolData();
    //初始化制定UART的DMA接收（huart7）
    Uart_InitUartDMA(Const_Communicate_UART_HANDLER); 
    //启动DMA接收到buscomm->uart_watchBuff缓冲区，长度为Const_Com_RX_BUFF_LEN（64）
    Uart_ReceiveDMA(Const_Communicate_UART_HANDLER, buscomm->uart_watchBuff, Const_Com_RX_BUFF_LEN);
}

// Check if protocol is offline, 调整buscomm的state状态
uint8_t Protocol_IsProtocolOffline() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    if (DWT_GetDeltaTWithOutUpdate(&buscomm->last_rx_tick) > Const_Comm_OFFLINE_TIME) {
        buscomm->state = Protocol_STATE_LOST;
    }
    return buscomm->state != Protocol_STATE_CONNECTED;
}

// 构造要发送给对端（nuc/上位机）的数据包并发送之（现在发送调用被注释）
void Protocol_SendProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
    uint32_t len;   
    // to nuc
    len = 4;
    buscomm->usb_sendBuff[0] = 0X5A;
    buscomm->usb_sendBuff[1] = 0XA5;
    for (int i = 0; i < Const_Protocol_Transmit_BUFF_SIZE; i++) {
        if (ProtocolCmd_Send[i].bus_func != NULL) {
            len += ProtocolCmd_Send[i].bus_func(buscomm->usb_sendBuff + len);
        }
    }
    buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
    buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
    buscomm->usb_sendBuff[len] = 0X7A;
    buscomm->usb_sendBuff[len + 1 ] = 0XA7;
    //Uart_SendMessage(Const_Communicate_UART_HANDLER, buscomm->usb_sendBuff, len + 2, 10);
    // Usb_SendBuff(buscomm->usb_sendBuff, 64);
}

// 无任何设置，仅返回16，
static uint32_t _set_Platform_Data_(uint8_t *buff) {

    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

    if(rc->remote.s[0] == Remote_SWITCH_DOWN && rc->remote.s[1] == Remote_SWITCH_UP ){
    //Platform_Set_Cal_LTO_PC(buff2float(buff),buff2float(buff+4),buff2float(buff+8),buff2float(buff+12));
    }
    return 16;
}

// 设置云台数据
static uint32_t _set_Gimbal_Data_(uint8_t *buff){
	
	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
	
	gimbal->Pole_Yaw_ref = buff2float(buff);
	gimbal->Pole_Pitch_ref = buff2float(buff + 4);

	Gimbal_SetPos();
	return 8;
}

// 合并并验证接收到的数据包(buff[] 为收到的数据，rxdatalen 为收到的数据长度)
static uint8_t Protocol_MergeAndverify(uint8_t buff[], uint32_t rxdatalen) {

    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    const uint8_t hdr0 = 0x5A, hdr1 = 0xA5;
    const uint8_t tail0 = 0x7A, tail1 = 0xA7;

    /* 允许外部传入新的数据块（如 DMA 本次接收的数据），也允许调用者仅检查累积缓冲中是否有完整帧
       当 buff==NULL 或 rxdatalen==0 时，表示仅从累积缓冲尝试提取已经存在的完整帧 */
    if (buff != NULL && rxdatalen > 0) {
        /* 防止累积缓冲溢出 */
        if (rx_accum_len + rxdatalen > PROTOCOL_RX_ACCUM_SIZE) {
            /* 溢出处理：丢弃已有数据并重置累积缓冲 */
            rx_accum_len = 0;
            return 1; /* 未能得到完整帧 */
        }
        memcpy(rx_accum + rx_accum_len, buff, rxdatalen);
        rx_accum_len += rxdatalen;
    }

    /* 在累积缓冲中查找完整帧 */
    uint32_t i = 0;
    while (i + 4 <= rx_accum_len) { /* 至少能包含头和长度字段 */
        /* 查找帧头 */
        if (!(rx_accum[i] == hdr0 && rx_accum[i + 1] == hdr1)) {
            i++;
            continue;
        }

        /* 至少要有 4 字节才能读取长度 */
        if (i + 4 > rx_accum_len) break; /* 等待更多数据 */

        uint32_t L = (uint32_t)rx_accum[i + 2] | ((uint32_t)rx_accum[i + 3] << 8); /* length field（与发送端一致） */

        /* 基本合法性检查：L 应该至少为 4（头+len）或更大；我们需要完整帧长度为 L + 2（尾巴2字节） */
        if (L < 4) {
            /* 非法长度，跳过这个头，继续查找下一个可能的头 */
            i++;
            continue;
        }

        uint32_t frame_total = L + 2; /* 包含尾部两个字节 */

        /* 是否收到完整帧 */
        if (i + frame_total > rx_accum_len) {
            /* 不完整，等待更多数据 */
            break;
        }

        /* 验证尾部标志 */
        if (rx_accum[i + L] == tail0 && rx_accum[i + L + 1] == tail1) {
            /* 找到完整且校验通过的帧：拷贝到 uart_watchBuff 供后续解析使用 */
            uint32_t copy_len = frame_total;
            if (copy_len > Const_Com_RX_BUFF_LEN) {
                /* 接收到的单帧过大，直接丢弃该帧以避免越界 */
                /* 移除该帧并继续查找下一个 */
                uint32_t remove = i + frame_total;
                if (remove < rx_accum_len) {
                    memmove(rx_accum, rx_accum + remove, rx_accum_len - remove);
                }
                rx_accum_len -= remove;
                return 1;
            }

            memcpy(buscomm->uart_watchBuff, rx_accum + i, copy_len);

            /* 删除已处理的字节（包括头到尾） */
            uint32_t remove = i + frame_total;
            if (remove < rx_accum_len) {
                memmove(rx_accum, rx_accum + remove, rx_accum_len - remove);
            }
            rx_accum_len -= remove;

            return 0; /* 成功返回，表示 uart_watchBuff 中存有一帧可供处理 */
        } else {
            /* 尾部不匹配，可能是误检头，跳过当前头继续查找 */
            i++;
            continue;
        }
    }

    /* 若查找过程中丢弃了前导无效字节，移位缓冲 */
    if (i > 0 && i < rx_accum_len) {
        memmove(rx_accum, rx_accum + i, rx_accum_len - i);
        rx_accum_len -= i;
    } else if (i >= rx_accum_len) {
        /* 没有发现有效头，清空缓冲以节省空间 */
        rx_accum_len = 0;
    }

    return 1; /* 没有完全可用帧 */
}

// uint8_t lkn[100];

// Decode received data packet and update bus data
void Protocol_DecodeData(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    /* 尝试把本次收到的数据并入累积缓冲并提取一帧（若可用） */
    uint8_t res = Protocol_MergeAndverify(buff, rxdatalen);

    /* 可能存在多帧已累积，循环处理直到没有完整帧为止 */
    while (res == 0) {
        /* 更新接收时间戳 */
        buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);

        uint32_t len = 4; /* payload 从 offset 4 开始（0-1 头，2-3 length） */
        for (int i = 0; i < Const_Protocol_Receive_BUFF_SIZE; i++) {
            if (ProtocolCmd_Receive[i].bus_func != NULL) {
                /* 每个回调应返回其消耗的字节数，解析时以此累加偏移 */
                uint32_t consumed = ProtocolCmd_Receive[i].bus_func(buscomm->uart_watchBuff + len);
                /* 边界检查：防止回调返回异常值导致越界 */
                if (consumed == 0) {
                    /* 回调表项可能表示空操作或解析失败，继续下一个 */
                    continue;
                }
                len += consumed;
                /* 简单保护：如果 len 超过缓冲区大小则中止解析本帧 */
                if (len > Const_Com_RX_BUFF_LEN) {
                    break;
                }
            }
        }

        /* 尝试提取下一个已准备好的帧（若存在），传入 NULL 表示仅检查累积缓冲 */
        res = Protocol_MergeAndverify(NULL, 0);
    }
}
Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE] = {
    // { _send_Gimbal_data},
    {_send_Remote_data},
    {_send_Platform_data},
};

// 两个数组按协议约定顺序把“分段”发送/接收处理函数排列起来
Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE] = {
     {&_set_Platform_Data_},
	 {&_set_Gimbal_Data_}
};

// 发送遥控器数据
// 遥控器数据打包格式见periph_remote.c中Remote_UpdateRemoteData函数注释
// 把遥控器（Remote）数据按字节序写入 buff 中,并返回11（写入的字节数）
static uint32_t _send_Remote_data(uint8_t *buff) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    buff[0] = (uint8_t)(rc->remote.s[0]) | ((uint8_t)(rc->remote.s[1]) << 4);
    buff[1] = (uint8_t)(rc->remote.ch[0] >> 8);
    buff[2] = (uint8_t)(rc->remote.ch[0]);
    buff[3] = (uint8_t)(rc->remote.ch[1] >> 8);
    buff[4] = (uint8_t)(rc->remote.ch[1]);
    buff[5] = (uint8_t)(rc->remote.ch[2] >> 8);
    buff[6] = (uint8_t)(rc->remote.ch[2]);
    buff[7] = (uint8_t)(rc->remote.ch[3] >> 8);
    buff[8] = (uint8_t)(rc->remote.ch[3]);
    buff[9] = (uint8_t)(rc->remote.ch[4] >> 8);
    buff[10] =(uint8_t)(rc->remote.ch[4]);

    return 11;
}

// 发送平台数据
// 平台数据打包格式见module_platform.h中Platform_FeedbackTypeDef结构体注释
// Platform->fdb.front_pitch (offset 0)
// Platform->fdb.left_pitch (offset +4)
// Platform->fdb.right_pitch (offset +8)
// Platform->fdb.front_yaw (offset +12)
// Platform->fdb.left_yaw (offset +16)
// Platform->fdb.right_yaw (offset +20)
static uint32_t _send_Platform_data(uint8_t *buff) {
    
Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
 
     float2buff( Platform->fdb.front_pitch_angle, buff);
     float2buff( Platform->fdb.left_pitch_angle, buff+4);
     float2buff( Platform->fdb.right_pitch_angle, buff+8);
     float2buff( Platform->fdb.front_yaw_angle, buff+12);
     float2buff( Platform->fdb.left_yaw_angle, buff+16);
     float2buff( Platform->fdb.right_yaw_angle, buff+20);

    return 24;
}

// 解析云台目标角度（从 buff 中读取两个 float），把它们设置到 gimbal 对象并调用 Gimbal_SetPos()。
static uint32_t _send_Gimbal_data(uint8_t *buff) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
    float2buff( gimbal->Yaw_fdb, buff);
    float2buff( gimbal->Pitch_fdb, buff+4);
    return 8;
}


void Communicate_RXCallback(UART_HandleTypeDef* huart) {
    Protocol_DataTypeDef *com = Protocol_GetBusDataPtr();

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_Com_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    
	Protocol_DecodeData(com->uart_watchBuff,rxdatalen);
	
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Com_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}


