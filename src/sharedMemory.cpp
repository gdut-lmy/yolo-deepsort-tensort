//
// Created by lmy on 2022/6/1
//
#include "sharedMemory.h"


sharedMemory::sharedMemory(int sem1Value, char *name1, int sem2Value, char *name2, key_t key, int size,
                           int flag) : sem1(sem1Value, name1), sem2(sem2Value, name2),
                                       m_key(key), m_size(size), m_flag(flag), m_shmid(0), m_ptr(nullptr) {
}


void *sharedMemory::sharedMemoryInit(const void *buf, int shmFlag) {


    getShmid();
    getShmat(buf, shmFlag);
    return m_ptr;
}

void *sharedMemory::getMemoryAddress() {
    return m_ptr;
}

void sharedMemory::sharedMemoryDestroy(int cmd, struct shmid_ds *buf) {
    getShmctl(cmd, buf);
}

int sharedMemory::getShmid() {

    m_shmid = shmget(m_key, m_size, m_flag);

    if (m_shmid == -1) {
        perror("shmget failed");
        return -1;
    } else {
        return m_shmid;
    }

}


bool sharedMemory::getShmat(const void *buf, int shmFlag) {

    m_ptr = shmat(m_shmid, buf, shmFlag);
    if (m_ptr == (void *) -1) {
        perror("Shmat error");
        return false;
    } else {
        return true;
    }

}


bool sharedMemory::getShmctl(int cmd, struct shmid_ds *buf) const {
    int ret;
    ret = shmctl(m_shmid, cmd, buf);
    if (ret == -1) {
        perror("shmctl error");
        return false;

    } else {
        return true;
    }

}

void sharedMemory::writeData(const char *str, size_t len) {

    assert(str);
    memset(m_ptr, 0, m_size);
    sem2.wait();
    std::copy(str, str + len, (char *) m_ptr);
    sem1.post();
}

void sharedMemory::writeData(const std::string &str) {
    writeData(str.data(), str.length());
}

void sharedMemory::writeData(const void *data, size_t len) {
    assert(data);
    writeData(static_cast<const char *>(data), len);
}

char *sharedMemory::readData() {

    sem1.wait();
    sem2.post();
    return static_cast<char *> (m_ptr);
}

size_t sharedMemory::readData(char *buf) {

    return 0;
}


sharedMemory::~sharedMemory() = default;

Sem::Sem(int semValue, char *name, int flag, mode_t mode) : m_value(semValue), m_name(name), m_flag(flag),
                                                            m_mode(mode) {
    m_sem = sem_open(name, m_flag, m_mode, m_value);
    if (m_sem == SEM_FAILED) {
        perror("sem open error");
    }

}

Sem::~Sem() {

    sem_close(m_sem);
    sem_unlink(m_name);
}

void Sem::wait() {
    sem_wait(m_sem);
}

void Sem::post() {
    sem_post(m_sem);
}

