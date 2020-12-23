#ifndef __SEMAPHORE_H
#define __SEMAPHORE_H

struct semaphore
{
    mutex_t  mutex;
    event_t  event;
    uint32_t value;
};

typedef struct semaphore sem_t;

bool sem_init(sem_t *sem, uint32_t value);
void sem_delete(sem_t *sem);
void sem_post(sem_t *sem);
void sem_wait(sem_t *sem);
bool sem_timed_wait(sem_t *sem, uint32_t timeout);

#endif
