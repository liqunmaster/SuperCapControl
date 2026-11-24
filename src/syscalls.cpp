/**
 * @brief newlib 非阻塞串口重定向与有界堆实现。
 * @file syscalls.cpp
 */
#include <sys/stat.h>

#include <cerrno>

#include "uart.hpp"

extern "C"
{
        /**
         * @brief 将标准输出或错误输出交给 UART DMA 队列。
         * @param fd 文件描述符，仅允许 1 或 2。
         * @param data 输入字节缓冲。
         * @param length 请求字节数。
         * @return 实际入队字节数，可短写；错误时返回 -1 并设置 errno。
         *
         */
        int _write(int fd, const char* data, int length)
        {
                if (__get_IPSR() != 0) {
                        errno = EWOULDBLOCK;
                        return -1;
                }
                if (fd != 1 && fd != 2) {
                        errno = EBADF;
                        return -1;
                }
                if (length < 0) {
                        errno = EINVAL;
                        return -1;
                }
                if (length == 0) {
                        return 0;
                }
                const auto written = drivers::uart::instance().write(data, static_cast<unsigned>(length));
                if (written == 0) {
                        errno = EAGAIN;
                        return -1;
                }
                return static_cast<int>(written);
        }
        /**
         * @brief 从 UART 接收队列非阻塞读取标准输入。
         * @param fd 文件描述符，仅允许 0。
         * @param[out] data 接收缓冲。
         * @param length 缓冲容量。
         * @return 实际字节数；无数据或参数错误返回 -1 并设置 errno。
         *
         */
        int _read(int fd, char* data, int length)
        {
                if (fd != 0) {
                        errno = EBADF;
                        return -1;
                }
                if (length < 0 || data == nullptr) {
                        errno = EINVAL;
                        return -1;
                }
                int count = 0;
                uint8_t byte;
                while (count < length && drivers::uart::instance().read(byte)) {
                        data[count++] = static_cast<char>(byte);
                }
                if (count == 0 && length != 0) {
                        errno = EAGAIN;
                        return -1;
                }
                return count;
        }
        /**
         * @brief 将标准流标记为字符设备。
         * @param fd 标准流描述符，范围为 0 至 2。
         * @param[out] status 有效的状态结构指针。
         * @return 成功返回 0，描述符无效返回 -1。
         *
         */
        int _fstat(int fd, struct stat* status)
        {
                if (fd < 0 || fd > 2) {
                        errno = EBADF;
                        return -1;
                }
                status->st_mode = S_IFCHR;
                return 0;
        }
        /**
         * @brief 判断文件描述符是否为标准串口流。
         * @param fd 文件描述符。
         * @return 描述符在 0 至 2 内返回 1，否则返回 0。
         *
         */
        int _isatty(int fd)
        {
                return fd >= 0 && fd <= 2;
        }
        /**
         * @brief 拒绝关闭标准串口流。
         * @return 固定返回 -1，并设置 EBADF。
         *
         */
        int _close(int)
        {
                errno = EBADF;
                return -1;
        }
        /**
         * @brief 拒绝对串口流进行定位。
         * @return 固定返回 -1，并设置 ESPIPE。
         *
         */
        int _lseek(int, int, int)
        {
                errno = ESPIPE;
                return -1;
        }
        /**
         * @brief 使用链接脚本预留的 2 KiB 堆为 newlib 提供内存。驱动使用静态缓冲区。
         * @param increment 非负的申请字节数，不支持缩减堆。
         * @return 原堆顶地址；超出范围时返回 (void*)-1 并设置 ENOMEM。
         *
         */
        void* _sbrk(int increment)
        {
                extern char _end;
                static char* current = &_end;
                if (increment < 0 || increment > 0x800 - (current - &_end)) {
                        errno = ENOMEM;
                        return reinterpret_cast<void*>(-1);
                }
                char* previous = current;
                current += increment;
                return previous;
        }
}
