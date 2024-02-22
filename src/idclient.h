#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include<netdb.h>
#include <unistd.h>

#include <stdarg.h>

#include <cstring>
#include <cmath>

#include <thread>
#include <chrono>

#include <vector>
#include <string>
#include <atomic>
#include <mutex>

struct SocketProp
{
    const char* host;
    int port;
    int protocol;// IPPROTO_TCP or IPPROTO_UDP
    bool verbose;
};

enum SocketClientStatus { SUCCESS, ALREADY_CONNECTED, ALREADY_DISCONNECTED, ERROR };

class SocketClient
{
private:
    SocketProp prop_;
    int sockFd_;
    struct sockaddr_in stSockAddr_;
    std::mutex lock_;
    std::atomic<bool> isConnF_;
    std::atomic<bool> exitF_;

private:
    void _logger(const char* fmt, ...)
    {
        if (this->prop_.verbose)
        {
            double timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000000.0;
            char printBuf[1024];
            sprintf(printBuf, "[%.9lf] %s", timestamp, fmt);
            va_list args;
            va_start(args, fmt);
            vprintf(printBuf, args);
            va_end(args);
        }
    }

public:
    SocketClient(const SocketProp& prop) : sockFd_(-1), isConnF_(false), exitF_(false)
    {
        try
        {
            this->prop_ = prop;
            // Open socket.
            this->sockFd_ = socket(PF_INET, SOCK_STREAM, this->prop_.protocol);
            // set socket timeout to 500ms
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 500000;
            setsockopt(this->sockFd_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
            setsockopt(this->sockFd_, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

            if (this->sockFd_ == -1)
            {
                this->_logger("[SocketClient] Open socket error.\n");
                throw "[SocketClient] Open socket error.";
            }
            // Filled struct.
            memset(&this->stSockAddr_, 0, sizeof(struct sockaddr_in));
            this->stSockAddr_.sin_family = AF_INET;
            this->stSockAddr_.sin_port = htons(this->prop_.port);
            if (inet_pton(AF_INET, this->prop_.host, &this->stSockAddr_.sin_addr) <= 0)
            {
                close(this->sockFd_);
                this->_logger("[SocketClient] Fill struct error.\n");
                throw "[SocketClient] Fill struct error.";
            }
        }
        catch (const char* msg)
        {
            throw msg;
        }
        catch (...)
        {
            this->_logger("[SocketClient] Unknow exception.\n");
            throw "[SocketClient] Unknow exception.";
        }
    }

    ~SocketClient()
    {
        this->disconnect();
    }

    SocketClientStatus connect()
    {
        std::lock_guard<std::mutex> locker(this->lock_);
        if (this->isConnF_)
            return SocketClientStatus::ALREADY_CONNECTED;
        try
        {
            if (::connect(this->sockFd_, (const struct sockaddr *)&this->stSockAddr_, sizeof(struct sockaddr_in)) < 0)
                return SocketClientStatus::ERROR;
        }
        catch (...)
        {
            return SocketClientStatus::ERROR;
        }
        this->isConnF_ = true;
        this->_logger("[SocketClient::connect] Connected to server.\n");
        return SocketClientStatus::SUCCESS;
    }

    SocketClientStatus disconnect()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return ALREADY_DISCONNECTED;
        this->exitF_ = true;// All looping process will be braked if exitF_ set to true.

        std::lock_guard<std::mutex> locker(this->lock_);
        this->isConnF_ = false;
        try
        {
            if (this->sockFd_ != -1)
            {
                shutdown(this->sockFd_, SHUT_RDWR);
                close(this->sockFd_);
            }
            this->_logger("[SocketClient::disconnect] Disconnected from server.\n");
            return SocketClientStatus::SUCCESS;
        }
        catch (...)
        {
            return SocketClientStatus::ERROR;
        }
    }

    SocketClientStatus send(const unsigned char* msg, int size)
    {
        std::lock_guard<std::mutex> locker(this->lock_);
        try
        {
            if (this->prop_.verbose)
            {
                // Show first 32 bytes of message in hex and formed into string using sprintf.
                char printBuf[256];
                int i = 0;
                for (i = 0; i < 48 && i < size; i++)
                    sprintf(printBuf + i * 3, "%02X ", msg[i]);
                if (i < size)
                    sprintf(printBuf + i * 3, "...");
                this->_logger("[SocketClient::send] Start send message: %s\n", printBuf);
            }
            if (::send(this->sockFd_, msg, size, MSG_DONTWAIT) < 0)
                return SocketClientStatus::ERROR;
            this->_logger("[SocketClient::send] Message sent.\n");
        }
        catch(...)
        {
            return SocketClientStatus::ERROR;
        }
        return SocketClientStatus::SUCCESS;
    }

    /** 
     * Socket read.
     * Function will not alloc recvBuf. The size of recvBuf must equal or greater then prop.readBufferSize.
     */
    SocketClientStatus recv(unsigned char* recvBuf, int size)
    {
        std::lock_guard<std::mutex> locker(this->lock_);
        try
        {
            this->_logger("[SocketClient::recv] Start recv.\n");
            if (::recv(this->sockFd_, recvBuf, size, 0) <= 0)
                return SocketClientStatus::ERROR;
            this->_logger("[SocketClient::recv] Received message: %s\n", recvBuf);
        }
        catch(...)
        {
            return SocketClientStatus::ERROR;
        }
        return SocketClientStatus::SUCCESS;
    }

    SocketClientStatus flushRecvBuffer()
    {
        std::lock_guard<std::mutex> locker(this->lock_);
        try
        {
            this->_logger("[SocketClient::flushRecvBuffer] Start flush recv buffer.\n");
            unsigned char recvBuf[1024];
            if (::recv(this->sockFd_, recvBuf, 1024, MSG_DONTWAIT) < 0)
                return SocketClientStatus::ERROR;
            this->_logger("[SocketClient::flushRecvBuffer] End flush recv buffer.\n");
        }
        catch(...)
        {
            return SocketClientStatus::ERROR;
        }
        return SocketClientStatus::SUCCESS;
    }
};


struct JimIDClientProp
{
    const char* host;
    unsigned char controllerId;
    std::vector<unsigned char> driveMotorIdVec;
    std::vector<unsigned char> steeringMotorIdVec;
    bool verbose;
};

class JimIDClient
{
private:
    SocketClient *testSock_;// Port 10002.
    SocketClient *aliveSock_;// Por 10003.
    SocketClient *dataSock_;// Port 10004.
    std::mutex testSockLock_;// Lock testSock_.
    std::mutex aliveSockLock_;// Lock aliveSock_.
    std::mutex dataSockLock_;// Lock dataSock_.

    std::atomic<bool> isConnF_;

    const JimIDClientProp prop_;

    std::thread *sendAliveTh_;
    std::thread *flushRecvBufferTh_;
    std::atomic<bool> exitF_;

private:
    void _logger(const char* fmt, ...)
    {
        if (this->prop_.verbose)
        {
            double timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000000.0;
            char printBuf[1024];
            sprintf(printBuf, "[%.9lf] %s", timestamp, fmt);
            va_list args;
            va_start(args, fmt);
            vprintf(printBuf, args);
            va_end(args);
        }
    }

    unsigned char* _packMsg(unsigned char* src, int size)
    {
        unsigned char* dst = new unsigned char[size + 4];
        dst[0] = size & 0xFF;
        dst[1] = (size >> 8) & 0xFF;
        dst[2] = (size >> 16) & 0xFF;
        dst[3] = (size >> 24) & 0xFF;
        memcpy(dst + 4, src, size);
        return dst;
    }

    void _sendAliveTh()
    {
        std::unique_lock<std::mutex> locker(this->aliveSockLock_, std::defer_lock);
        while (!this->exitF_)
        {
            this->_logger("[JimIDClient::_sendAliveTh] Start send alive signal.\n");
            std::vector<unsigned char> tmp = { 0x42, 0x42, 0x42, 0x42 };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            locker.lock();
            this->aliveSock_->send(buf, tmp.size() + 4);
            locker.unlock();
            delete buf;
            this->_logger("[JimIDClient::_sendAliveTh] End send alive signal.\n");
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void _flushRecvBufferTh()
    {
        std::unique_lock<std::mutex> aliveSockLocker(this->aliveSockLock_, std::defer_lock);
        std::unique_lock<std::mutex> dataSockLocker(this->dataSockLock_, std::defer_lock);
        std::unique_lock<std::mutex> testSockLocker(this->testSockLock_, std::defer_lock);
        while (!this->exitF_)
        {
            this->_logger("[JimIDClient::_flushRecvBufferTh] Start flush recv buffer.\n");
            testSockLocker.lock();
            this->testSock_->flushRecvBuffer();
            testSockLocker.unlock();
            aliveSockLocker.lock();
            this->aliveSock_->flushRecvBuffer();
            aliveSockLocker.unlock();
            dataSockLocker.lock();
            this->dataSock_->flushRecvBuffer();
            dataSockLocker.unlock();
            this->_logger("[JimIDClient::_flushRecvBufferTh] End flush recv buffer.\n");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

public:
    JimIDClient(const JimIDClientProp& prop) : 
        prop_(prop), 
        testSock_(nullptr), 
        aliveSock_(nullptr), 
        dataSock_(nullptr), 
        isConnF_(false), 
        sendAliveTh_(nullptr), 
        flushRecvBufferTh_(nullptr), 
        exitF_(false)
    {
        
    }

    ~JimIDClient()
    {
        this->close();
    }

    bool connect()
    {
        if (this->isConnF_ || this->exitF_)
            return false;
        std::unique_lock<std::mutex> testSockLocker(this->testSockLock_, std::defer_lock);
        std::unique_lock<std::mutex> aliveSockLocker(this->aliveSockLock_, std::defer_lock);
        std::unique_lock<std::mutex> dataSockLocker(this->dataSockLock_, std::defer_lock);

        this->_logger("[JimIDClient::connect] Create socket.\n");

        testSockLocker.lock();
        aliveSockLocker.lock();
        dataSockLocker.lock();
        try
        {
            this->testSock_ = new SocketClient({ this->prop_.host, 10002, IPPROTO_TCP, this->prop_.verbose });
            this->aliveSock_ = new SocketClient({ this->prop_.host, 10003, IPPROTO_TCP, this->prop_.verbose });
            this->dataSock_ = new SocketClient({ this->prop_.host, 10004, IPPROTO_TCP, this->prop_.verbose });
        }
        catch(...)
        {
            if (this->testSock_ != nullptr)
            {
                delete this->testSock_;
                this->testSock_ = nullptr;
            }
            if (this->aliveSock_ != nullptr)
            {
                delete this->aliveSock_;
                this->aliveSock_ = nullptr;
            }
            if (this->dataSock_ != nullptr)
            {
                delete this->dataSock_;
                this->dataSock_ = nullptr;
            }
            this->_logger("[JimIDClient::connect] Create socket error.\n");
            dataSockLocker.unlock();
            aliveSockLocker.unlock();
            testSockLocker.unlock();
            return false;
        }

        this->_logger("[JimIDClient::connect] Connect to server.\n");
        bool connF = true;
        connF &= this->testSock_->connect() != SocketClientStatus::ERROR;
        connF &= this->aliveSock_->connect() != SocketClientStatus::ERROR;
        connF &= this->dataSock_->connect() != SocketClientStatus::ERROR;

        if (connF)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x03, 0x01, 0x00, 0x00, this->prop_.controllerId };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            this->_logger("[JimIDClient::connect] Send controller id.\n");
            connF &= this->testSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS;
            connF &= this->aliveSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS;
        }

        dataSockLocker.unlock();
        aliveSockLocker.unlock();
        testSockLocker.unlock();

        if (connF)
        {
            this->_logger("[JimIDClient::connect] Start alive thread.\n");
            this->sendAliveTh_ = new std::thread(&JimIDClient::_sendAliveTh, this);
            this->_logger("[JimIDClient::connect] Start flush recv buffer thread.\n");
            this->flushRecvBufferTh_ = new std::thread(&JimIDClient::_flushRecvBufferTh, this);
            this->isConnF_ = true;
        }
        return this->isConnF_;
    }

    /**
     * Send drive motor signal.
     * @param[in] vec: The drive motor signal vector, which element is the signed pwm of the motor.
     * @param[in] prk: The drive motor parking signal vector.
     * @return true if send success. Otherwise, return false.
     */
    bool sendDriveMotorSignal(std::vector<double> vec, std::vector<bool> prk)
    {
        if (!this->isConnF_ || vec.size() != prk.size() || vec.size() != this->prop_.driveMotorIdVec.size())
            return false;
        std::lock_guard<std::mutex> dataSockLocker(this->dataSockLock_);
        bool ret = true;

        for (int i = 0; i < vec.size(); i++)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x04, 0x01, 0x00, 0x04, 
                                                static_cast<unsigned char>(vec[i] < 0 ? 1 : 2), 
                                                static_cast<unsigned char>(prk[i] ? 1 : 0), 
                                                static_cast<unsigned char>(abs(vec[i])), 
                                                this->prop_.driveMotorIdVec[i] };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            this->_logger("[JimIDClient::sendDriveMotorSignal] Send drive motor signal: %d\n", this->prop_.driveMotorIdVec[i]);
            ret &= this->dataSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS;
            delete buf;
            if (!ret)
            {
                this->isConnF_ = false;
                break;
            }
            this->_logger("[JimIDClient::sendDriveMotorSignal] Signal sent.\n");
        }
        return ret;
    }

    /**
     * Send steering motor signal.
     * @param[in] vec: The steering motor signal vector, which element is the distance of the motor.
     * @return true if send success. Otherwise, return false.
     */
    bool sendSteeringMotorSignal(std::vector<double> vec)
    {
        if (!this->isConnF_ || vec.size() != this->prop_.steeringMotorIdVec.size())
            return false;
        std::lock_guard<std::mutex> dataSockLocker(this->dataSockLock_);
        bool ret = true;

        for (int i = 0; i < vec.size(); i++)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02, 
                                                static_cast<unsigned char>((int)(vec[i]) / 256), 
                                                static_cast<unsigned char>((int)(vec[i]) % 256), 
                                                this->prop_.steeringMotorIdVec[i] };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            ret &= this->dataSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS;
            delete buf;
            if (!ret)
            {
                this->isConnF_ = false;
                break;
            }
            this->_logger("[JimIDClient::sendSteeringMotorSignal] Signal sent.\n");
        }
        return ret;
    }

    bool isConnected() {  return this->isConnF_; }

    void close()
    {
        if (this->exitF_)
            return;
        this->exitF_ = true;

        std::lock_guard<std::mutex> testSockLocker(this->testSockLock_);
        std::lock_guard<std::mutex> aliveSockLocker(this->aliveSockLock_);
        std::lock_guard<std::mutex> dataSockLocker(this->dataSockLock_);
        this->isConnF_ = false;

        this->_logger("[JimIDClient::close] Join send alive thread.\n");
        if (this->sendAliveTh_ != nullptr)
        {
            this->sendAliveTh_->join();
            delete this->sendAliveTh_;
        }
        this->_logger("[JimIDClient::close] Join send alive thread done.\n");

        this->_logger("[JimIDClient::close] Join flush recv buffer thread.\n");
        if (this->flushRecvBufferTh_ != nullptr)
        {
            this->flushRecvBufferTh_->join();
            delete this->flushRecvBufferTh_;
        }
        this->_logger("[JimIDClient::close] Join flush recv buffer thread done.\n");

        this->_logger("[JimIDClient::close] Disconnecting socket.\n");
        if (this->testSock_ != nullptr)
        {
            delete this->testSock_;
            this->testSock_ = nullptr;
        }
        if (this->aliveSock_ != nullptr)
        {
            delete this->aliveSock_;
            this->aliveSock_ = nullptr;
        }
        if (this->dataSock_ != nullptr)
        {
            delete this->dataSock_;
            this->dataSock_ = nullptr;
        }
        this->_logger("[JimIDClient::close] Socket deleted.\n");
    }
};
