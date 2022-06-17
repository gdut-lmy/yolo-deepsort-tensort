//
// Created by lmy on 2022/6/16.
//

#ifndef SHM_SHAREDMEMORY_H
#define SHM_SHAREDMEMORY_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <string>
#include <cstring>
#include <memory>
#include <cassert>
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "unistd.h"


class sharedMemory {
public:
    sharedMemory(key_t key, int size, int flag = 0666 | IPC_CREAT);

    void *sharedMemoryInit(const void *buf, int shmFlag);

    void *getMemoryAddress();

    void sharedMemoryDestroy(int cmd, struct shmid_ds *buf);

    void writeData(const char *str, size_t len);

    void writeData(const std::string &str);

    void writeData(const void *data, size_t len);

    char *readData();

    size_t readData(char *buf);

    sharedMemory(const sharedMemory &) = delete;

    sharedMemory &operator=(const sharedMemory &) = delete;

    ~sharedMemory();


private:
    int getShmid();

    bool getShmat(const void *buf, int shmFlag);

    bool getShmctl(int cmd, struct shmid_ds *buf) const;

private:
    key_t m_key;//为共享内存段命名，多个共享同一片内存的进程使用同一个key
    int m_size;//共享内存容量
    int m_flag;//权限标志位
    int m_shmid;//共享内存ID
    void *m_ptr;//得到的共享内存首地址

};

//进程同步信号量
class Sem {

public:
    Sem(int sem1Value, char *name, int flag = O_CREAT, mode_t mode = 0666);

    ~Sem();

    void wait();

    void post();

private:

    int m_flag;
    char *m_name;
    unsigned int m_value;
    mode_t m_mode;
    sem_t *m_sem;
};


#endif //SHM_SHAREDMEMORY_H
