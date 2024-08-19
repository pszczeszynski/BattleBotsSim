#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
#include <hidclass.h>

int rawhid_open(int max, int vid, int pid, int usage_page, int usage);
int rawhid_recv(int num, void *buf, int len, int timeout);
int rawhid_send(int num, void *buf, int len, int timeout);
void rawhid_close(int num);

class RawHID
{
public:
    RawHID();
    void Initialize(HANDLE handle);
    bool IsOpen();
    void Close();

    bool SendAsync(void *buf, int len);
    int CheckSendAsync();
    bool IsSendPending();

    bool RecvAsync(int len);
    int CheckRecvAsync(int len, void *buf);
    bool IsRecvPending();
    bool ResetRecv();

private:
    HANDLE _handle;
    bool _open;
    HANDLE _rx_event;
    HANDLE _tx_event;

    bool _tx_in_progress;
    bool _rx_in_progress;

    OVERLAPPED _tx_ov;
    OVERLAPPED _rx_ov;

    uint8_t _tx_buf[516];
    uint8_t _rx_buf[516];
};

int RawHID_Open(RawHID *hid_list, int max, int vid, int pid, int usage_page, int usage);

