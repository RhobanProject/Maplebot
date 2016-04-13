#include <stdlib.h>
#include <wirish/wirish.h>
#include <string.h>
#include "dxl_gy85.h"
#include "dxl_protocol.h"
#include "coder.h"

#define BUFFERS 5

struct gy85_values
{
    unsigned int sequence;
    int accelerometer;
    int gyroscope;
    int magnetometer;
};

struct gy85
{
    i2c_dev *dev;
    HardwareSerial *port;
    int state;
    int lastUpdate;
    unsigned char checksum;
    struct gy85_values data[5];
    int pos;
    struct dxl_registers registers;
};

void gy85_init(struct gy85 *gy85)
{
    i2c_init(gy85->dev);
    i2c_master_enable(gy85->dev, I2C_FAST_MODE);
}

void gy85_begin(struct gy85 *gy85, i2c_dev *dev)
{
    int i;
    for (i=0; i<BUFFERS; i++) {
        gy85->data[i].sequence = 0;
    }
    gy85->pos = 0;
    gy85->state = 0;
    gy85->dev = dev;
    gy85->port = port;
    gy86->lastUpdate = millis();

    gy85_init(gy85);
}

void gy85_update(struct gy85 *gy85)
{
}

void gy85_tick(struct gy85 *gy85)
{
    int now = millis();
    int delta = now-gy85->lastUpdate;
    if (delta > 10) {
        gy85->lastUpdate = now;
        gy85_update(gy85);
    } else if (delta < 0) {
        gy85->lastUpdate = millis();
    }
};


static bool dxl_gy85_check_id(volatile struct dxl_device *self, ui8 id)
{
    struct gy85 *gy85 = (struct gy85*)self->data;
    return (id == gy85->registers.eeprom.id);
}

static void dxl_gy85_write_data(volatile struct dxl_device *self, ui8 id, ui8 addr, ui8 *values, ui8 length)
{
    struct gy85 *gy85 = (struct gy85*)self->data;
    memcpy(((ui8 *)(&gy85->registers))+addr, values, length);
}

static void dxl_gy85_read_data(volatile struct dxl_device *self, ui8 id, ui8 addr, ui8 *values, ui8 length, ui8 *error)
{
    struct gy85 *gy85 = (struct gy85*)self->data;

    if (addr == IMU_ADDR) {
        memcpy(values, ((ui8*)&gy85->data), length);
    } else {
        memcpy(values, ((ui8*)&gy85->registers)+addr, length);
    }
    *error = 0;
}

static void dxl_gy85_process(volatile struct dxl_device *self, volatile struct dxl_packet *packet)
{
    dxl_process(self, packet, dxl_gy85_check_id, dxl_gy85_write_data, dxl_gy85_read_data);
}

static void gy85_tick(volatile struct dxl_device *self)
{
    struct gy85 *gy85 = (struct gy85*)self->data;
    gy85_tick(gy85);
}

void dxl_gy85_init(volatile struct dxl_device *device, ui8 id, i2c_dev *dev)
{
    struct gy85 *gy85 = (struct gy85 *)malloc(sizeof(struct gy85));
    dxl_device_init(device);
    device->process = dxl_gy85_process;
    device->data = (void *)gy85;
    device->tick = gy85_tick;

    gy85_begin(gy85, port);

    gy85->registers.eeprom.modelNumber = DXL_GY85_MODEL;
    gy85->registers.eeprom.firmwareVersion = 1;
    gy85->registers.eeprom.id = id;
    gy85->registers.ram.presentPosition = 0;
    gy85->registers.ram.goalPosition = 0;
    gy85->registers.ram.torqueEnable = 0;
    gy85->registers.ram.torqueLimit = 0;
}
