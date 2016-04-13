#include <wirish/wirish.h>
#include <dma.h>
#include <usart.h>
#include <usb_cdcacm.h>
#include "dxl_protocol.h"
#include "dxl_serial.h"

// Serial com
#define DIRECTION_485   31
#define DIRECTION_TTL   30

static void receiveMode();

static dma_tube_config tube_config;
volatile static bool txComplete = true;
volatile static bool dmaEvent = false;
volatile char dmaBuffer[DXL_BUFFER_SIZE];

HardwareTimer syncReadTimer(3);
bool syncReadMode = false;
ui8 syncReadAddr = 0;
ui8 syncReadLength = 0;
ui8 syncReadIds[100];
ui8 syncReadCount = 0;
ui8 syncReadCurrent = 0;
struct dxl_packet syncReadPacket;

static void DMAEvent()
{
    dmaEvent = true;
}

static void setupSerial3DMA(int n)
{
    // We're receiving from the USART data register. serial->c_dev()
    // returns a pointer to the libmaple usart_dev for that serial
    // port, so this is a pointer to its data register.
    tube_config.tube_dst = &Serial3.c_dev()->regs->DR;
    // We're only interested in the bottom 8 bits of that data register.
    tube_config.tube_dst_size = DMA_SIZE_8BITS;
    // We're storing to rx_buf.
    tube_config.tube_src = dmaBuffer;
    // rx_buf is a char array, and a "char" takes up 8 bits on STM32.
    tube_config.tube_src_size = DMA_SIZE_8BITS;
    // Only fill BUF_SIZE - 1 characters, to leave a null byte at the end.
    tube_config.tube_nr_xfers = n;
    // Flags:
    // - DMA_CFG_DST_INC so we start at the beginning of rx_buf and
    // fill towards the end.
    // - DMA_CFG_CIRC so we go back to the beginning and start over when
    // rx_buf fills up.
    // - DMA_CFG_CMPLT_IE to turn on interrupts on transfer completion.
    tube_config.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE | DMA_CFG_ERR_IE;
    // Target data: none. It's important to set this to NULL if you
    // don't have any special (microcontroller-specific) configuration
    // in mind, which we don't.
    tube_config.target_data = NULL;
    // DMA request source.
    tube_config.tube_req_src = DMA_REQ_SRC_USART3_TX;
}

static void receiveMode()
{
    asm volatile("nop");
    // Disabling transmitter
    Serial3.enableTransmitter(false);

    // Sets the direction to receiving
    digitalWrite(DIRECTION_TTL, HIGH);
    digitalWrite(DIRECTION_485, LOW);

    // Enabling the receiver
    Serial3.enableReceiver(true);
    asm volatile("nop");
}

void transmitMode()
{
    asm volatile("nop");
    // Disabling the receiver
    Serial3.enableReceiver(false);

    // Sets the direction to transmitting
    digitalWrite(DIRECTION_TTL, LOW);
    digitalWrite(DIRECTION_485, HIGH);

    // Enabling the transmitter
    // pinMode(Serial3.txPin(), PWM);
    Serial3.enableTransmitter(true);
    asm volatile("nop");
}

void initSerial3(int baudrate = DXL_DEFAULT_BAUDRATE)
{
    pinMode(DIRECTION_TTL, OUTPUT);
    pinMode(DIRECTION_485, OUTPUT);

    // Enabling Serial3 in receive mode
    Serial3.begin(baudrate);
    Serial3.c_dev()->regs->CR3 = USART_CR3_DMAT;
    // Serial3.enableHalfDuplex();
    receiveMode();
}

void sendSerialPacket(volatile struct dxl_packet *packet)
{
    // We have a packet for the serial bus
    // First, clear the serial input buffers
    Serial3.flush();

    // Then, send it
    int n = dxl_write_packet(packet, (ui8 *)dmaBuffer);

    // Go in transmit mode
    transmitMode();

    // Then runs the DMA transfer
    dmaEvent = false;
    txComplete = false;
    setupSerial3DMA(n);
    dma_init(DMA1);
    dma_tube_cfg(DMA1, DMA_CH2, &tube_config);
    dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_VERY_HIGH);
    dma_attach_interrupt(DMA1, DMA_CH2, DMAEvent);
    dma_enable(DMA1, DMA_CH2);

    /*
    char buffer[1024];
    n = dxl_write_packet(packet, (ui8 *)buffer);
    for (int i=0; i<n; i++) {
        Serial3.write(buffer[i]);
    }
    Serial3.waitDataToBeSent();
    receiveMode();
    */
}
          
/**
 * Sends the next syn read packet
 */
void syncReadSendPacket()
{
    struct dxl_packet readPacket;
    readPacket.instruction = DXL_READ_DATA;
    readPacket.parameter_nb = 2;
    readPacket.id = syncReadIds[syncReadCurrent];
    readPacket.parameters[0] = syncReadAddr;
    readPacket.parameters[1] = syncReadLength;
    sendSerialPacket(&readPacket);

    syncReadPacket.dxl_state = 0;
    syncReadPacket.process = false;
}

/**
 * Ticking
 */
static void tick(volatile struct dxl_device *self) 
{
    static int baudrate = DXL_DEFAULT_BAUDRATE;

    if (syncReadMode) {
        if (!txComplete) {
            if (dmaEvent) {
                txComplete = true;
                syncReadTimer.refresh();
                Serial3.waitDataToBeSent();
                dma_disable(DMA1, DMA_CH2);
                receiveMode();
            }
        } else {
            bool processed = false;

            while (Serial3.available() && !self->packet.process) {
                dxl_packet_push_byte(&syncReadPacket, Serial3.read());
            }
            if (syncReadPacket.process && syncReadPacket.parameter_nb == syncReadLength) {
                ui8 i;
                self->packet.parameters[syncReadCurrent*(syncReadLength+1)] = syncReadPacket.error;
                for (i=0; i<syncReadLength; i++) {
                    self->packet.parameters[syncReadCurrent*(syncReadLength+1)+1+i] = syncReadPacket.parameters[i];
                }
                processed = true;
            }
            if (syncReadTimer.getCount() > 550) {
                self->packet.parameters[syncReadCurrent*(syncReadLength+1)] = 0xff;
                processed = true;
            }
            if (processed) {
                syncReadCurrent++;
                if (syncReadCurrent >= syncReadCount) {
                    self->packet.process = true;
                    syncReadMode = false;
                } else {
                    syncReadSendPacket();
                }
            }
        }
    } else {
        // When DMA has completed its work
        if (!txComplete) {
            if (dmaEvent) {
                txComplete = true;
                dmaEvent = false;
                Serial3.waitDataToBeSent();
                dma_disable(DMA1, DMA_CH2);
                receiveMode();
            }
        } else {
            if (self->redirect_packets == NULL && baudrate != usb_cdcacm_get_baud()) {
    //            baudrate = usb_cdcacm_get_baud();
    //            initSerial3(baudrate);
            }

            while (Serial3.available() && !self->packet.process) {
                dxl_packet_push_byte(&self->packet, Serial3.read());
            }
        }
    }
}

/**
 * Processing master packets (sending it to the local bus)
 */
static void process(volatile struct dxl_device *self, volatile struct dxl_packet *packet)
{
    tick(self);

    if (txComplete && !syncReadMode) {
        if (packet->instruction == DXL_SYNC_READ && packet->parameter_nb > 2) {
            ui8 i;
            syncReadMode = true;
            syncReadAddr = packet->parameters[0];
            syncReadLength = packet->parameters[1];

            syncReadCount = 0;
            for (i=0; (i+2)<packet->parameter_nb; i++) {
                syncReadCount++;
                syncReadIds[i] = packet->parameters[i+2];
            }

            syncReadCurrent = 0;
            syncReadSendPacket();

            self->packet.error = 0;
            self->packet.parameter_nb = (syncReadLength+1)*syncReadCount;
            self->packet.process = false;
            self->packet.id = packet->id;
        } else {
            if (packet->id == DXL_BROADCAST || packet->id < 200) {
                self->packet.dxl_state = 0;
                self->packet.process = false;
                sendSerialPacket(packet);
            }
        }
    }
}

void dxl_serial_init(volatile struct dxl_device *device)
{
    dxl_device_init(device);
    device->tick = tick;
    device->process = process;

    initSerial3();

    syncReadTimer.pause();
    syncReadTimer.setPrescaleFactor(CYCLES_PER_MICROSECOND);
    syncReadTimer.setOverflow(0xffff);
    syncReadTimer.refresh();
    syncReadTimer.resume();
}
