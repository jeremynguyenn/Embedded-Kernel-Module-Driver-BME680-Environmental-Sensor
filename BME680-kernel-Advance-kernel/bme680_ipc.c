// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/uaccess.h>
#include <linux/ipc.h> // Bổ sung cho System V IPC
#include "bme680.h"

struct bme680_ipc_data {
    struct sock *nl_sk;
    struct bme680_data *data;
    int sysv_msgid; // Bổ sung: System V message queue ID
};

static struct bme680_ipc_data *ipc_data;

// Cấu trúc cho System V message queue
struct bme680_sysv_msg {
    long mtype; // Loại thông điệp
    char mtext[64]; // Nội dung thông điệp
};

void bme680_send_netlink_alert(struct bme680_data *data, const char *msg) {
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    int res;

    skb = nlmsg_new(strlen(msg) + 1, GFP_KERNEL);
    if (!skb) return;

    nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, strlen(msg) + 1, 0);
    if (!nlh) {
        kfree_skb(skb);
        return;
    }

    strcpy(nlmsg_data(nlh), msg);
    res = nlmsg_multicast(ipc_data->nl_sk, skb, 0, 1, GFP_KERNEL);
    if (res < 0) kfree_skb(skb);
}

// Bổ sung: Gửi thông báo qua System V message queue
void bme680_send_sysv_alert(struct bme680_data *data, const char *msg) {
    struct bme680_sysv_msg sysv_msg;
    sysv_msg.mtype = 1; // Loại thông điệp cố định
    strncpy(sysv_msg.mtext, msg, sizeof(sysv_msg.mtext) - 1);
    sysv_msg.mtext[sizeof(sysv_msg.mtext) - 1] = '\0';

    if (msgsnd(ipc_data->sysv_msgid, &sysv_msg, sizeof(sysv_msg.mtext), IPC_NOWAIT) < 0) {
        pr_err("bme680: Failed to send System V message\n");
    }
}

void bme680_check_threshold(struct bme680_data *data) {
    struct bme680_fifo_data fdata;
    if (kfifo_peek(&data->data_fifo, &fdata)) {
        if (fdata.gas_resistance > data->gas_threshold) {
            char msg[64];
            snprintf(msg, sizeof(msg), "Gas resistance exceeded threshold: %u Ohms", fdata.gas_resistance);
            bme680_send_netlink_alert(data, msg);
            bme680_send_sysv_alert(data, msg); // Bổ sung: Gửi qua System V
            iio_push_event(data->indio_dev, IIO_EVENT_CODE(IIO_RESISTANCE, 0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
                           ktime_get_ns());
        }
    }
}

int bme680_ipc_init(struct bme680_data *data) {
    struct netlink_kernel_cfg cfg = {
        .input = NULL,
    };
    key_t sysv_key = 1234; // Khóa System V IPC

    ipc_data = kzalloc(sizeof(*ipc_data), GFP_KERNEL);
    if (!ipc_data) return -ENOMEM;

    ipc_data->data = data;
    ipc_data->nl_sk = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
    if (!ipc_data->nl_sk) {
        kfree(ipc_data);
        return -ENOMEM;
    }

    // Bổ sung: Khởi tạo System V message queue
    ipc_data->sysv_msgid = msgget(sysv_key, IPC_CREAT | 0666);
    if (ipc_data->sysv_msgid < 0) {
        pr_err("bme680: Failed to create System V message queue\n");
        netlink_kernel_release(ipc_data->nl_sk);
        kfree(ipc_data);
        return -ENOMEM;
    }

    return 0;
}

void bme680_ipc_cleanup(struct bme680_data *data) {
    if (ipc_data) {
        if (ipc_data->nl_sk) {
            netlink_kernel_release(ipc_data->nl_sk);
        }
        if (ipc_data->sysv_msgid >= 0) { // Bổ sung: Xóa System V message queue
            msgctl(ipc_data->sysv_msgid, IPC_RMID, NULL);
        }
        kfree(ipc_data);
    }
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BME680 IPC Driver");
MODULE_VERSION("3.2");