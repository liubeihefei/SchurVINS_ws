#include "USART.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>

USART::USART(long buff_size)
        : fd(0), buffer(new char[buff_size]), buffer_size(buff_size){

}

USART::USART(const std::string &usart_file, int bound_rate, long buff_size, bool block, int recv_wait_time,
             int recv_min_char)
             : fd(0), buffer(new char[buff_size]), buffer_size(buff_size){
    init(usart_file, bound_rate, block, recv_wait_time, recv_min_char);
}

USART::~USART()
{
    delete[] buffer;
    if (fd > 0)
        close(fd);
}

bool USART::init(const std::string &usart_file, int bound_rate, bool block, int recv_wait_time,
                 int recv_min_char) noexcept(false) {
    auto flag = block ? (O_RDWR | O_NOCTTY) : (O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(usart_file.c_str(), flag);
    if (-1 == fd)
        throw std::runtime_error("USART: Serial port open error!");

    struct serial_struct serial{};
    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &serial);

    /// 测试是否为终端设备
    if (isatty(fd) == 0)
        throw std::runtime_error("USART: Standard input is not a terminal device\n");
    struct termios newtio{}, oldtio{};
    /// 保存现有串口参数设置，如果出错可以回溯log
    if (tcgetattr(fd, &oldtio) != 0)
        throw std::runtime_error("USART: tcgetattr(fd, &oldtio) -> %d");
    /// 第一步，设置字符大小-8位
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    /// 设置奇偶校验位-不校验
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = 0;

    /// 设置波特率
    switch (bound_rate) {
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 1000000:
            cfsetispeed(&newtio, B1000000);
            cfsetospeed(&newtio, B1000000);
            break;
        case 3000000:
            cfsetispeed(&newtio, B3000000);
            cfsetospeed(&newtio, B3000000);
            break;
        default:
            throw std::invalid_argument("USART: not a valid boundrate arguments");
    }
    /// 设置停止位
    newtio.c_cflag &= ~CSTOPB;
    /// 设置等待时间和最小接受字符
    newtio.c_cc[VTIME] = recv_wait_time;
    newtio.c_cc[VMIN] = recv_min_char;
    /// 处理未接收字符
    tcflush(fd, TCIFLUSH);
    /// 激活新配置
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        throw std::runtime_error("USART: tcsetattr error!");
    return true;
}

std::string USART::Recv() {
    auto ret = read(fd, buffer, buffer_size);
    std::string message(ret, 0);
    memcpy((char *) message.data(), buffer, ret);
    return message;
}

/// 按位设置0，1，后续标示位可以用这个来进行标注
static void setByteFlags(unsigned char &byte,
                         int start_index,
                         int len,
                         bool flag) {
    auto temp = static_cast<unsigned char>((1 << len) - 1);
    if (flag)
        // 置 1
        byte |= temp << start_index;
    else
        // 置 0
        byte &= ~(temp << start_index);
}

/// 串口发送函数
bool USART::Send(const std::string &message) const {
    auto ret = write(fd, message.data(), message.length());
    return ret == message.length();
}

// bool USART::Send(SendPack &send_pack) const {
//     bool res = false;
//     Frame frame;
//     int data_length = 9;
//     /// 帧头
//     frame[0] = 0x3c;
//     /// 数据帧
//     frame[1] = static_cast<int16_t>(send_pack.x)>>8;
//     frame[2] = static_cast<int16_t>(send_pack.x);
//     frame[3] = static_cast<int16_t>(send_pack.y)>>8;
//     frame[4] = static_cast<int16_t>(send_pack.y);
//     frame[5] = static_cast<int16_t>(send_pack.z)>>8;
//     frame[6] = static_cast<int16_t>(send_pack.z);
//     std::cout << "y di:" << static_cast<int16_t>(frame[4]) << std::endl;
//     std::cout << "y gao:" << static_cast<int16_t>(frame[3]) << std::endl;
//     std::cout << "z di:" << static_cast<int16_t>(frame[6]) << std::endl;
//     std::cout << "z gao:" << static_cast<int16_t>(frame[5]) << std::endl;
//     frame[7] = send_pack.work_flag;
//     /// 帧尾
//     frame[8] = 0x3d;
//     long nbytes = write(fd, frame.data(), 9);
//     // std::cout << "nbytes:" << nbytes << std::endl;
//     if (nbytes < 0) {
//        std::cout << "USART: send failed." << std::endl;
//        res = false;
//     } else if (nbytes == 9)
//     {
//         res = true;
//     }
//     return res;
// }

typedef union
{
    float a;
    uint8_t b[4];
} float2char_u;

bool USART::Send(SendPack &send_pack) const {
    /// 原：先发高八位再发低八位
    bool res = false;
    Frame frame;
    int data_length = 30;
    int fronthalf = 37;
    int cmd_id = 0x0302;
    /// 帧头5字节、指令id两字节、数据30字节、帧尾2字节
    /// 帧头
    /// 数据帧起始字节
    frame[0] = 0xA5;
    send_pack.head.SOF = 0xA5;
    /// 数据帧data长度，这里是30
    frame[2] = static_cast<int16_t>(data_length)>>8;
    frame[1] = static_cast<int16_t>(data_length);
    send_pack.head.data_length = data_length;
    /// 包序号
    // frame[3] = 0x02;
    frame[3] = 0x5c;
    send_pack.head.seq = 0x5c;
    /// 帧头CRC8校验
    send_pack.head.CRC8 = Get_CRC8_Check_Sum((std::uint8_t *)&frame,
                                     4, CRC8_INIT);
    frame[4] = send_pack.head.CRC8;
    // frame[4] = 0x05;
    /// 指令ID
    frame[6] = static_cast<int16_t>(cmd_id)>>8;
    frame[5] = static_cast<int16_t>(cmd_id);
    /// 数据帧
    /// 坐标
    float2char_u cvt;
    cvt.a = send_pack.x;
    frame[8] = cvt.b[0];
    frame[9] = cvt.b[1];
    frame[10] = cvt.b[2];
    frame[11] = cvt.b[3];

    cvt.a = send_pack.y;
    frame[12] = cvt.b[0];
    frame[13] = cvt.b[1];
    frame[14] = cvt.b[2];
    frame[15] = cvt.b[3];

    cvt.a = send_pack.z;
    frame[16] = cvt.b[0];
    frame[17] = cvt.b[1];
    frame[18] = cvt.b[2];
    frame[19] = cvt.b[3];

    cvt.a = send_pack.yaw;
    frame[20] = cvt.b[0];
    frame[21] = cvt.b[1];
    frame[22] = cvt.b[2];
    frame[23] = cvt.b[3];

    cvt.a = send_pack.pitch;
    frame[24] = cvt.b[0];
    frame[25] = cvt.b[1];
    frame[26] = cvt.b[2];
    frame[27] = cvt.b[3];

    cvt.a = send_pack.roll;
    frame[28] = cvt.b[0];
    frame[29] = cvt.b[1];
    frame[30] = cvt.b[2];
    frame[31] = cvt.b[3];

    /// 自定义控制器工作状况
    frame[32] = send_pack.work_flag;
    // frame[19] = 0xED;
    /// 剩下用0补齐
    frame[33] = 0;
    frame[34] = 0;
    frame[35] = 0;
    frame[36] = 0;
    /// 帧尾
    /// CRC16
    send_pack.tail = Get_CRC16_Check_Sum((std::uint8_t *) &frame, fronthalf, CRC_INIT);
    frame[38] = static_cast<int16_t>(send_pack.tail)>>8;
    frame[37] = static_cast<int16_t>(send_pack.tail);
    long nbytes = write(fd, frame.data(), 39);
    // std::cout << "nbytes:" << nbytes;
    if (nbytes < 0) {
        std::cout << "USART: send failed." << std::endl;
        res = false;
    } else if (nbytes == 39){
        res = true;
    }
    return res;
}



