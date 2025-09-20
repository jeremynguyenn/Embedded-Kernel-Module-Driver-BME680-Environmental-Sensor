#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <pthread.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <linux/seccomp.h>
#include <linux/filter.h>
#include <sys/prctl.h>
#include <string.h>
#include <math.h>
#include <mqueue.h>
#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/select.h>
#include <poll.h>
#include <sys/epoll.h>
#include <sched.h>
#include <time.h>
#include <sys/time.h>
#include <sys/file.h>
#include "bme680.h"

#define SHM_NAME "/bme680_shm"
#define PIPE_BUF_SIZE 1024
#define MAX_READS 100
#define NETLINK_USER 31
#define IAQ_GOOD 0
#define IAQ_MODERATE 1
#define IAQ_POOR 2
#define GAS_THRESHOLD 100000
#define FIFO_NAME "/tmp/bme680_fifo"
#define SEM_NAME "/bme680_sem"
#define MQ_NAME "/bme680_mq"

volatile sig_atomic_t keep_running = 1;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t data_cond = PTHREAD_COND_INITIALIZER;
pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;
pthread_barrier_t barrier;
struct bme680_fifo_data *shared_data;
int shm_fd, nl_sock, fifo_fd;
mqd_t mq;
sem_t *sem;
struct mq_attr mq_attr = { .mq_flags = 0, .mq_maxmsg = 10, .mq_msgsize = sizeof(struct bme680_fifo_data), .mq_curmsgs = 0 };
key_t sysv_key = 1234;
int sysv_shmid, sysv_semid, sysv_msgid;
struct sembuf sysv_semop = { .sem_num = 0, .sem_op = -1, .sem_flg = 0 };
void *original_brk;

// Cấu trúc cho System V message queue
struct bme680_sysv_msg {
    long mtype;
    struct bme680_fifo_data data;
};

void demo_heap_management() {
    original_brk = sbrk(0);
    sbrk(1024);
    printf("Heap extended by 1024 bytes\n");
    brk(original_brk);
}

void demo_memory_management(void *buf, size_t size) {
    mprotect(buf, size, PROT_READ);
    madvise(buf, size, MADV_SEQUENTIAL);
    mlock(buf, size);
    munlock(buf, size);
}

void demo_string_funcs(char *str) {
    char *token = strtok(str, ",");
    char buf[100];
    sprintf(buf, "Token: %s\n", token);
    printf("%s", buf);
}

void *read_thread(void *arg) {
    int dev_fd = open("/dev/bme680", O_RDONLY | O_NONBLOCK);
    if (dev_fd < 0) {
        perror("Failed to open /dev/bme680");
        return NULL;
    }

    struct bme680_fifo_data *reads = malloc(MAX_READS * sizeof(struct bme680_fifo_data));
    if (!reads) {
        close(dev_fd);
        return NULL;
    }

    int num_reads = *(int *)arg;
    while (keep_running && (num_reads == 0 || num_reads-- > 0)) {
        struct bme680_fifo_data fdata;
        if (ioctl(dev_fd, BME680_IOC_READ_FIFO, &fdata) >= 0) {
            pthread_mutex_lock(&data_mutex);
            memcpy(shared_data, &fdata, sizeof(fdata));
            pthread_cond_signal(&data_cond);
            pthread_mutex_unlock(&data_mutex);

            // Bổ sung: Gửi dữ liệu qua POSIX message queue
            if (mq_send(mq, (char *)&fdata, sizeof(fdata), 0) < 0) {
                perror("mq_send failed");
            }

            // Bổ sung: Gửi dữ liệu qua FIFO
            if (write(fifo_fd, &fdata, sizeof(fdata)) < 0) {
                perror("FIFO write failed");
            }

            // Bổ sung: Gửi dữ liệu qua System V message queue
            struct bme680_sysv_msg sysv_msg = { .mtype = 1, .data = fdata };
            if (msgsnd(sysv_msgid, &sysv_msg, sizeof(sysv_msg.data), IPC_NOWAIT) < 0) {
                perror("System V msgsnd failed");
            }
        }
        usleep(1000 * 1000); // 1s
    }

    free(reads);
    close(dev_fd);
    return NULL;
}

void *process_thread(void *arg) {
    while (keep_running) {
        pthread_mutex_lock(&data_mutex);
        pthread_cond_wait(&data_cond, &data_mutex);
        struct bme680_fifo_data local_data;
        pthread_rwlock_rdlock(&rwlock);
        memcpy(&local_data, shared_data, sizeof(local_data));
        pthread_rwlock_unlock(&rwlock);
        pthread_mutex_unlock(&data_mutex);

        // Bổ sung: Đồng bộ hóa bằng POSIX semaphore
        sem_wait(sem);
        printf("Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%, Gas: %u Ohms\n",
               local_data.temperature / 100.0, local_data.pressure / 100.0,
               local_data.humidity / 1000.0, local_data.gas_resistance);
        sem_post(sem);
    }
    return NULL;
}

void *netlink_thread(void *arg) {
    struct sockaddr_nl sa = { .nl_family = AF_NETLINK, .nl_groups = 1 };
    nl_sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_USER);
    if (nl_sock < 0) {
        perror("netlink socket failed");
        return NULL;
    }
    if (bind(nl_sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("netlink bind failed");
        close(nl_sock);
        return NULL;
    }

    char buf[256];
    while (keep_running) {
        ssize_t len = recv(nl_sock, buf, sizeof(buf), 0);
        if (len > 0) {
            printf("Netlink alert: %s\n", buf);
        }
    }
    close(nl_sock);
    return NULL;
}

void *sysv_msg_thread(void *arg) { // Bổ sung: Thread nhận System V message
    struct bme680_sysv_msg sysv_msg;
    while (keep_running) {
        if (msgrcv(sysv_msgid, &sysv_msg, sizeof(sysv_msg.data), 1, 0) >= 0) {
            printf("System V Message - Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%, Gas: %u Ohms\n",
                   sysv_msg.data.temperature / 100.0, sysv_msg.data.pressure / 100.0,
                   sysv_msg.data.humidity / 1000.0, sysv_msg.data.gas_resistance);
        } else {
            perror("System V msgrcv failed");
        }
    }
    return NULL;
}

void signal_handler(int sig, siginfo_t *info, void *context) {
    if (sig == SIGINT || sig == SIGTERM) {
        keep_running = 0;
    } else if (sig == SIGUSR1) {
        printf("Received SIGUSR1 with value: %d\n", info->si_value.sival_int);
    }
}

int main(int argc, char *argv[]) {
    int opt, interval_ms = 1000, num_reads = 0, use_sysfs = 0;
    struct option long_options[] = {
        {"interval", required_argument, 0, 'i'},
        {"num-reads", required_argument, 0, 'n'},
        {"sysfs", no_argument, 0, 's'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "i:n:s", long_options, NULL)) != -1) {
        switch (opt) {
        case 'i': interval_ms = atoi(optarg); break;
        case 'n': num_reads = atoi(optarg); break;
        case 's': use_sysfs = 1; break;
        default: exit(1);
        }
    }

    // Khởi tạo POSIX semaphore
    sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        return 1;
    }

    // Khởi tạo POSIX message queue
    mq = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &mq_attr);
    if (mq == (mqd_t)-1) {
        perror("mq_open failed");
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    // Khởi tạo FIFO
    if (mkfifo(FIFO_NAME, 0666) < 0 && errno != EEXIST) {
        perror("mkfifo failed");
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }
    fifo_fd = open(FIFO_NAME, O_WRONLY | O_NONBLOCK);
    if (fifo_fd < 0) {
        perror("FIFO open failed");
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    // Khởi tạo System V IPC
    sysv_shmid = shmget(sysv_key, sizeof(struct bme680_fifo_data), IPC_CREAT | 0666);
    if (sysv_shmid < 0) {
        perror("shmget failed");
        close(fifo_fd);
        unlink(FIFO_NAME);
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    sysv_semid = semget(sysv_key, 1, IPC_CREAT | 0666);
    if (sysv_semid < 0) {
        perror("semget failed");
        shmctl(sysv_shmid, IPC_RMID, NULL);
        close(fifo_fd);
        unlink(FIFO_NAME);
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    sysv_msgid = msgget(sysv_key, IPC_CREAT | 0666);
    if (sysv_msgid < 0) {
        perror("msgget failed");
        shmctl(sysv_shmid, IPC_RMID, NULL);
        semctl(sysv_semid, 0, IPC_RMID);
        close(fifo_fd);
        unlink(FIFO_NAME);
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    // Khởi tạo POSIX shared memory
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        perror("shm_open failed");
        shmctl(sysv_shmid, IPC_RMID, NULL);
        semctl(sysv_semid, 0, IPC_RMID);
        msgctl(sysv_msgid, IPC_RMID, NULL);
        close(fifo_fd);
        unlink(FIFO_NAME);
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    ftruncate(shm_fd, sizeof(struct bme680_fifo_data));
    shared_data = mmap(NULL, sizeof(struct bme680_fifo_data), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_data == MAP_FAILED) {
        perror("mmap failed");
        shm_unlink(SHM_NAME);
        close(shm_fd);
        shmctl(sysv_shmid, IPC_RMID, NULL);
        semctl(sysv_semid, 0, IPC_RMID);
        msgctl(sysv_msgid, IPC_RMID, NULL);
        close(fifo_fd);
        unlink(FIFO_NAME);
        mq_close(mq);
        mq_unlink(MQ_NAME);
        sem_close(sem);
        sem_unlink(SEM_NAME);
        return 1;
    }

    // Khởi tạo barrier
    pthread_barrier_init(&barrier, NULL, 3);

    // Tạo các thread
    pthread_t read_tid, process_tid, nl_tid, sysv_msg_tid;
    pthread_create(&read_tid, NULL, read_thread, &num_reads);
    pthread_create(&process_tid, NULL, process_thread, NULL);
    pthread_create(&nl_tid, NULL, netlink_thread, NULL);
    pthread_create(&sysv_msg_tid, NULL, sysv_msg_thread, NULL); // Bổ sung: Thread System V

    // Chờ các thread hoàn thành
    pthread_join(read_tid, NULL);
    pthread_join(process_tid, NULL);
    pthread_join(nl_tid, NULL);
    pthread_join(sysv_msg_tid, NULL);

    // Dọn dẹp tài nguyên
    pthread_barrier_destroy(&barrier);
    munmap(shared_data, sizeof(struct bme680_fifo_data));
    shm_unlink(SHM_NAME);
    close(shm_fd);
    close(fifo_fd);
    unlink(FIFO_NAME);
    mq_close(mq);
    mq_unlink(MQ_NAME);
    sem_close(sem);
    sem_unlink(SEM_NAME);
    shmctl(sysv_shmid, IPC_RMID, NULL);
    semctl(sysv_semid, 0, IPC_RMID);
    msgctl(sysv_msgid, IPC_RMID, NULL);
    pthread_mutex_destroy(&data_mutex);
    pthread_cond_destroy(&data_cond);
    pthread_rwlock_destroy(&rwlock);

    if (use_sysfs) {
        int sysfs_interval = interval_ms;
        while (keep_running && (num_reads == 0 || num_reads-- > 0)) {
            FILE *temp_file = fopen("/sys/bus/iio/devices/iio:device0/in_temp_input", "r");
            FILE *press_file = fopen("/sys/bus/iio/devices/iio:device0/in_pressure_input", "r");
            FILE *humid_file = fopen("/sys/bus/iio/devices/iio:device0/in_humidityrelative_input", "r");
            FILE *gas_file = fopen("/sys/bus/iio/devices/iio:device0/in_resistance_input", "r");
            if (!temp_file || !press_file || !humid_file || !gas_file) {
                perror("Failed to open sysfs files");
                if (temp_file) fclose(temp_file);
                if (press_file) fclose(press_file);
                if (humid_file) fclose(humid_file);
                if (gas_file) fclose(gas_file);
                break;
            }
            float temp, press, humid, gas;
            fscanf(temp_file, "%f", &temp);
            fscanf(press_file, "%f", &press);
            fscanf(humid_file, "%f", &humid);
            fscanf(gas_file, "%f", &gas);
            printf("SysFS - Temperature: %.2f °C\n", temp / 1000.0);
            printf("SysFS - Pressure: %.2f hPa\n", press * 10.0);
            printf("SysFS - Humidity: %.2f %%\n", humid / 1000.0);
            printf("SysFS - Gas Resistance: %.0f Ohms\n", gas);
            fclose(temp_file);
            fclose(press_file);
            fclose(humid_file);
            fclose(gas_file);
            usleep(sysfs_interval * 1000);
        }
    }

    printf("Application terminated normally.\n");
    return 0;
}