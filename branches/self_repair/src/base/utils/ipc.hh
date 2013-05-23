/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef IPC_HH
#define IPC_HH

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include "lolmsg.h"
#include "bytequeue.h"

#define IPCLOLBUFFERSIZE 264 //=256 + 8 
#define IPCTXBUFFERSIZE 2048 
#define IPCBLOCKSIZE 256 

namespace IPC{

class Connection;
typedef void (*Callback)(const LolMessage *msg, void * connection, void * user_ptr);

class Connection
{
    public:
        Connection();
        ~Connection();
        inline void SetCallback(Callback c, void * u) {callback = c; user_data = u;}
        bool connected;

        bool SendData(const uint8_t type, uint8_t *data, int len);
        bool Start();
        
        void * ipc;
        int sockfds;
        sockaddr_in addr; 

        bool transmiting_thread_running;
        bool receiving_thread_running;

    private:
        pthread_t receiving_thread;
        pthread_t transmiting_thread;
        static void * Receiving(void *ptr);
        static void * Transmiting(void *ptr);
        LolParseContext parseContext;
        Callback callback;
        void * user_data;
        ByteQueue txq;
        uint8_t txbuffer[IPCTXBUFFERSIZE];
        pthread_mutex_t mutex_txq;

};

class IPC
{
    public:
        IPC();
        ~IPC();

        bool Start(const char *host,int port, bool server);
        void Stop();
        bool SendData(const uint8_t type, uint8_t *data, int len);
        bool SendData(const uint32_t dest, const uint8_t type, uint8_t * data, int len);
        int BrokenConnections();
        inline void SetCallback(Callback c, void * u) {callback = c; user_data = u;}
        inline bool Server(){return server;}
        std::vector<Connection*> *Connections(){ return &connections;}

    private:
        static void * Monitoring(void *ptr);
        static void * Listening(void *ptr);
        bool StartServer(int port);
        bool ConnectToServer(const char * host, int port);
        int RemoveBrokenConnections();

        int sockfd;
        bool server;
        int port;
        char* host;
        pthread_t monitor_thread;
        pthread_t listening_thread;
        std::vector<Connection*> connections;
        bool monitoring_thread_running;
        
        Callback callback;
        void * user_data;
};

}//end of namespace
#endif
