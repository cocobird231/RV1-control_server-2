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

public:
    SocketClient(const SocketProp& prop) : sockFd_(-1), isConnF_(false)
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
            throw "[SocketClient] Open socket error.";
        // Filled struct.
        memset(&this->stSockAddr_, 0, sizeof(struct sockaddr_in));
        this->stSockAddr_.sin_family = AF_INET;
        this->stSockAddr_.sin_port = htons(this->prop_.port);
        if (inet_pton(AF_INET, this->prop_.host, &this->stSockAddr_.sin_addr) <= 0)
        {
            close(this->sockFd_);
            throw "[SocketClient] Fill struct error.";
        }
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
        return SocketClientStatus::SUCCESS;
    }

    SocketClientStatus disconnect()
    {
        std::lock_guard<std::mutex> locker(this->lock_);
        this->isConnF_ = false;
        try
        {
            shutdown(this->sockFd_, SHUT_RDWR);
            close(this->sockFd_);
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
            if (::send(this->sockFd_, msg, size, MSG_DONTWAIT) < 0)
                return SocketClientStatus::ERROR;
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
            if (::recv(this->sockFd_, recvBuf, size, 0) <= 0)
                return SocketClientStatus::ERROR;
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
            unsigned char recvBuf[1024];
            ::recv(this->sockFd_, recvBuf, 1024, MSG_DONTWAIT);
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
    SocketClient *aliveSock_;
    SocketClient *dataSock_;

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
        while (!this->exitF_)
        {
            this->_logger("[JimIDClient::_sendAliveTh] Start send alive signal.\n");
            std::vector<unsigned char> tmp = { 0x42, 0x42, 0x42, 0x42 };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            this->aliveSock_->send(buf, tmp.size() + 4);
            delete buf;
            this->_logger("[JimIDClient::_sendAliveTh] End send alive signal.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void _flushRecvBufferTh()
    {
        while (!this->exitF_)
        {
            this->_logger("[JimIDClient::_flushRecvBufferTh] Start flush recv buffer.\n");
            this->aliveSock_->flushRecvBuffer();
            this->dataSock_->flushRecvBuffer();
            this->_logger("[JimIDClient::_flushRecvBufferTh] End flush recv buffer.\n");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

public:
    JimIDClient(const JimIDClientProp& prop) : 
        prop_(prop), 
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
        this->_logger("[JimIDClient::connect] Create socket.\n");
        this->aliveSock_ = new SocketClient({ this->prop_.host, 10003, IPPROTO_TCP });
        this->dataSock_ = new SocketClient({ this->prop_.host, 10004, IPPROTO_TCP });

        this->_logger("[JimIDClient::connect] Connect to server.\n");
        if (this->aliveSock_->connect() != SocketClientStatus::ERROR && 
            this->dataSock_->connect() != SocketClientStatus::ERROR)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x03, 0x01, 0x00, 0x00, this->prop_.controllerId };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            this->_logger("[JimIDClient::connect] Send controller id.\n");
            if (this->aliveSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS && 
                this->dataSock_->send(buf, tmp.size() + 4) == SocketClientStatus::SUCCESS)
            {
                // Start alive thread.
                this->_logger("[JimIDClient::connect] Start alive thread.\n");
                this->sendAliveTh_ = new std::thread(&JimIDClient::_sendAliveTh, this);
                this->_logger("[JimIDClient::connect] Start flush recv buffer thread.\n");
                this->flushRecvBufferTh_ = new std::thread(&JimIDClient::_flushRecvBufferTh, this);
                this->isConnF_ = true;
            }
            else
            {
                this->_logger("[JimIDClient::connect] Start flush recv buffer thread.\n");
                this->isConnF_ = false;
            }
            delete buf;
        }
        return this->isConnF_;
    }

    /**
     * Send drive motor signal.
     * @param[in] vec: The drive motor signal vector, which element is the signed pwm of the motor.
     * @param[in] prk: The drive motor parking signal vector.
     * @return true if send success. Otherwise, return false.
     */
    bool sendDriveMotorSignal(std::vector<float> vec, std::vector<bool> prk)
    {
        if (!this->isConnF_ || vec.size() != prk.size() || vec.size() != this->prop_.driveMotorIdVec.size())
            return false;

        bool ret = true;

        for (int i = 0; i < vec.size(); i++)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x04, 0x01, 0x00, 0x04, 
                                                static_cast<unsigned char>(vec[i] < 0 ? 1 : 2), 
                                                static_cast<unsigned char>(prk[i] ? 1 : 0), 
                                                static_cast<unsigned char>(abs(vec[i])), 
                                                this->prop_.driveMotorIdVec[i] };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            ret &= this->dataSock_->send(buf, tmp.size() + 4);
            this->_logger("[JimIDClient::sendDriveMotorSignal] Send drive motor signal: %d\n", this->prop_.driveMotorIdVec[i]);
            delete buf;
        }
        return ret;
    }

    /**
     * Send steering motor signal.
     * @param[in] vec: The steering motor signal vector, which element is the distance of the motor.
     * @return true if send success. Otherwise, return false.
     */
    bool sendSteeringMotorSignal(std::vector<float> vec)
    {
        if (!this->isConnF_ || vec.size() != this->prop_.steeringMotorIdVec.size())
            return false;

        bool ret = true;

        for (int i = 0; i < vec.size(); i++)
        {
            std::vector<unsigned char> tmp = { 0x69, 0x74, 0x72, 0x69, 0x00, 0x08, 0x01, 0x00, 0x02, 
                                                static_cast<unsigned char>((int)(vec[i]) / 256), 
                                                static_cast<unsigned char>((int)(vec[i]) % 256), 
                                                this->prop_.steeringMotorIdVec[i] };
            auto buf = this->_packMsg(tmp.data(), tmp.size());
            ret &= this->dataSock_->send(buf, tmp.size() + 4);
            this->_logger("[JimIDClient::sendSteeringMotorSignal] Send steering motor signal: %d\n", this->prop_.steeringMotorIdVec[i]);
            delete buf;
        }
        return ret;
    }

    bool isConnected() {  return this->isConnF_; }

    void close()
    {
        if (!this->exitF_)
            this->exitF_ = true;
        else// Already exit.
            return;

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
        this->aliveSock_->disconnect();
        this->dataSock_->disconnect();
        this->_logger("[JimIDClient::close] Disconnecting socket done.\n");
        delete this->aliveSock_;
        delete this->dataSock_;
        this->_logger("[JimIDClient::close] Delete socket.\n");
    }
};
