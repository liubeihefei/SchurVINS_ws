//
// Created by yuan on 23-7-12.
//

#ifndef CAPIMG_USART_H
#define CAPIMG_USART_H

#include <string>
#include <array>
// #include "types.h"
#include "CRC.h"


class USART{
private:
    static constexpr unsigned char std_head = 0xfc;
    using DataBuffer = std::array<u_char, 100>;
public:
    using Frame = std::array<u_char, 39>;


    /// New Pack
    struct FrameHead
    {
        u_char SOF;
        int data_length;
        u_char seq;
        u_char CRC8;
    };

    struct SendPack
    {
        FrameHead head;
        float x = 0;
        float y = 0;
        float z = 0;
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        int tail;
        u_char work_flag;
    };


private:
    int fd;
    char *buffer;
    long buffer_size;
public:
    explicit USART(long buff_size = 1024);
    USART(const std::string &usart_file, int bound_rate, long buff_size = 1024,
          bool block = false, int recv_wait_time = 0, int recv_min_char = 0) noexcept(false);
    ~USART();
    bool init(const std::string &usart_file, int bound_rate, bool block = false, int recv_wait_time = 0, int recv_min_char = 0)
    noexcept(false);
    [[nodiscard]] inline bool is_init() const
    {
        return fd != 0;
    }
public:
    USART(USART &usart) = delete;
    USART(const USART &usart) = delete;
    const USART &operator = (USART &usart) = delete;
    const USART &operator = (const USART &usart) = delete;
public:
    std::string Recv();
    bool Send(const std::string &message) const;
    bool Send(SendPack &send_pack) const;
    bool Send(float value, int id) const; /* 重载函数,用来debug的函数 */
    // bool Receive(ReadPack &read_pack) const;

   

private:
    static int encode_data(const Frame &send_pack, DataBuffer &send_data);

};

#endif //CAPIMG_USART_H
