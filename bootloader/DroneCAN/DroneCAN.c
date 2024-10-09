/*
 * DroneCAN.c - support for DroneCAN protocol for ESC control and telemetry
 */

//#pragma GCC optimize("O0")

#if DRONECAN_SUPPORT

#include <version.h>
#include <eeprom.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sys_can.h"
#include <canard.h>
#include <blutil.h>
#include <DroneCAN.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include "dsdl_generated/dronecan_msgs.h"

#ifndef PREFERRED_NODE_ID
#define PREFERRED_NODE_ID 0
#endif

#ifndef CANARD_POOL_SIZE
#define CANARD_POOL_SIZE 4096
#endif

#ifndef DRONECAN_DEBUG
#define DRONECAN_DEBUG 0
#endif

// assume that main fw starts at 16k
#define MAIN_FW_START_ADDR 0x08004000

static CanardInstance canard;
static uint8_t canard_memory_pool[CANARD_POOL_SIZE];

#ifndef MCU_FLASH_START
#define MCU_FLASH_START 0x08000000
#endif

#if BOARD_FLASH_SIZE == 128
#define EEPROM_START_ADD (MCU_FLASH_START+0x1f800)
#else
#error "Only 128K flash size supported for DroneCAN"
#endif

/*
  keep the state for firmware update
*/
static struct {
    char path[256];
    uint8_t node_id;
    uint8_t transfer_id;
    uint32_t last_read_ms;
    uint32_t offset;
} fwupdate;

static bool have_raw_command;

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  hold our node status as a static variable. It will be updated on any errors
*/
static struct uavcan_protocol_NodeStatus node_status;

enum boot_code {
    CHECK_FW_OK = 0,
    FAIL_REASON_NO_APP_SIG = 10,
    FAIL_REASON_BAD_LENGTH_APP = 11,
    FAIL_REASON_BAD_BOARD_ID = 12,
    FAIL_REASON_BAD_CRC = 13,
    FAIL_REASON_IN_UPDATE = 14,
    FAIL_REASON_WATCHDOG = 15,
    FAIL_REASON_BAD_LENGTH_DESCRIPTOR = 16,
    FAIL_REASON_BAD_FIRMWARE_SIGNATURE = 17,
    FAIL_REASON_VERIFICATION = 18,
    FAIL_REASON_NO_SIGNAL = 19,
};

#define APP_SIGNATURE_MAGIC1 0x68f058e6
#define APP_SIGNATURE_MAGIC2 0xafcee5a0

/*
  application signature
 */
struct app_signature {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t fwlen; // total fw length in bytes
    uint32_t crc1; // crc32 up to start of app_signature
    uint32_t crc2; // crc32 from end of app_signature to end of fw
    char mcu[16];
    uint32_t unused[2];
};

/*
  simple 16 bit random number generator
*/
static uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific

  NOTE: this should be in functions.c
*/
static uint64_t micros64(void)
{
    static uint64_t base_us;
    static uint16_t last_cnt;
    uint16_t cnt = bl_timer_us();
    if (cnt < last_cnt) {
	base_us += 0x10000;
    }
    last_cnt = cnt;
    return base_us + cnt;
}

/*
  get monotonic time in milliseconds since startup
*/
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

/*
  default settings, based on public/assets/eeprom_default.bin in AM32 configurator
 */
static const uint8_t default_settings[] = {
    0x01, 0x02, 0x01, 0x01, 0x23, 0x4e, 0x45, 0x4f, 0x45, 0x53, 0x43, 0x20, 0x66, 0x30, 0x35, 0x31,
    0x20, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x18, 0x64, 0x37, 0x0e, 0x00, 0x00, 0x05, 0x00,
    0x80, 0x80, 0x80, 0x32, 0x00, 0x32, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x8d, 0x66, 0x06, 0x00, 0x00
};

// crc32 implementation, slow method for small flash cost
static uint32_t crc32(const uint8_t *buf, uint32_t size)
{
    uint32_t crc = 0;
    while (size--) {
        const uint8_t byte = *buf++;
        crc ^= byte;
        for (uint8_t i=0; i<8; i++) {
            const uint32_t mask = -(crc & 1);
            crc >>= 1;
            crc ^= (0xEDB88320 & mask);
        }
    }
    return crc;
}

// print to CAN LogMessage for debugging
static void can_print(const char *s)
{
    struct uavcan_protocol_debug_LogMessage pkt;
    memset(&pkt, 0, sizeof(pkt));

    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    pkt.text.len = strlen(s);
    memcpy(pkt.text.data, s, pkt.text.len);

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer);
    static uint8_t logmsg_transfer_id;

    canardBroadcast(&canard,
		    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
		    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
		    &logmsg_transfer_id,
		    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer, len);
}

/*
  handle parameter executeopcode request
*/
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = false;

    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        can_print("resetting to defaults");
        save_flash_nolib(default_settings, sizeof(default_settings), EEPROM_START_ADD);
        pkt.ok = true;
    }


    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle RestartNode request
*/
static void handle_RestartNode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    set_rtc_backup_register(0, 0);
    // reboot the ESC
    NVIC_SystemReset();
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = BOOTLOADER_VERSION;
    pkt.software_version.minor = 0;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    sys_can_getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char*)pkt.name.data, "AM32_BOOTLOADER_" AM32_MCU, sizeof(pkt.name.data));
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle a BeginFirmwareUpdate request from a management tool like
  DroneCAN GUI tool or MissionPlanner
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    /*
      decode the request
     */
    struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        return;
    }

    /*
      check for a repeated BeginFirmwareUpdateRequest
     */
    if (fwupdate.node_id == transfer->source_node_id &&
	memcmp(fwupdate.path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len) == 0) {
	/* ignore duplicate request */
	return;
    }

    fwupdate.offset = 0;
    fwupdate.node_id = transfer->source_node_id;
    strncpy(fwupdate.path, (char*)req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_file_BeginFirmwareUpdateResponse reply;
    memset(&reply, 0, sizeof(reply));
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);

    can_print("Started firmware update");
}

/*
  send a read for a firmware update. This asks the client (firmware
  server) for a piece of the new firmware
 */
static void send_firmware_read(void)
{
    uint32_t now = millis32();
    if (fwupdate.last_read_ms != 0 && now - fwupdate.last_read_ms < 750) {
        // the server may still be responding
        return;
    }
    fwupdate.last_read_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];

    struct uavcan_protocol_file_ReadRequest pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.path.path.len = strlen((const char *)fwupdate.path);
    pkt.offset = fwupdate.offset;
    memcpy(pkt.path.path.data, fwupdate.path, pkt.path.path.len);

    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
			   fwupdate.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
			   &fwupdate.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
  handle response to send_firmware_read()
 */
static void handle_file_read_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_id+1)%32 != fwupdate.transfer_id ||
	transfer->source_node_id != fwupdate.node_id) {
	/* not for us */
        can_print("Firmware update: not for us");
	return;
    }
    struct uavcan_protocol_file_ReadResponse pkt;
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &pkt)) {
	/* bad packet */
        can_print("Firmware update: bad packet");
	return;
    }
    if (pkt.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
        if (fwupdate.offset == 0) {
            // MissionPlanner does this sometimes, ignore
            return;
        }
	/* read failed */
	fwupdate.node_id = 0;
        can_print("Firmware update read failure");
	return;
    }

    uint32_t len = pkt.data.len;
    len = (len+7U) & ~7U;
    save_flash_nolib(pkt.data.data, len, (uint32_t)MAIN_FW_START_ADDR + fwupdate.offset);

    fwupdate.offset += pkt.data.len;

    if (pkt.data.len < 256) {
	/* firmware updare done */
        can_print("Firmwate update complete");
	fwupdate.node_id = 0;
        set_rtc_backup_register(0, 0);
        NVIC_SystemReset();
	return;
    }

    /* trigger a new read */
    fwupdate.last_read_ms = 0;
    send_firmware_read();

    DroneCAN_processTxQueue();
}

/*
  data for dynamic node allocation process
*/
static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

/*
  handle a DNA allocation packet
*/
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
	(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
	DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
	/* bad packet */
	return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    sys_can_getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
	DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
*/
static void request_DNA()
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
	(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[16];
    sys_can_getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(&canard,
		    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		    &node_id_allocation_transfer_id,
		    CANARD_TRANSFER_PRIORITY_LOW,
		    &allocation_request[0],
		    (uint16_t) (uid_size + 1));

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
  This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
            handle_RestartNode(ins, transfer);
            break;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    handle_begin_firmware_update(ins, transfer);
	    break;
	}
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            handle_param_ExecuteOpcode(ins, transfer);
            break;
        }
	}
    }
    if (transfer->transfer_type == CanardTransferTypeResponse) {
	switch (transfer->data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    handle_file_read_response(ins, transfer);
	    break;
        }
    }
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            handle_DNA_Allocation(ins, transfer);
            break;
        }
        }
    }
}


/*
  This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
  by the local node.
  If the callback returns true, the library will receive the transfer.
  If the callback returns false, the library will ignore the transfer.
  All transfers that are addressed to other nodes are always ignored.

  This function must fill in the out_data_type_signature to be the signature of the message.
*/
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
            return true;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
	    return true;
	}
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
	}
    }
    if (transfer_type == CanardTransferTypeResponse) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
	    return true;
	}
    }
    if (transfer_type == CanardTransferTypeBroadcast) {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
            // we are receiving RawCommand, we should boot to the main firmware
            have_raw_command = true;
        }
        }
    }
    // we don't want any other messages
    return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
*/
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
    node_status.sub_mode = 0;

    /*
      when doing a firmware update put the size in kbytes in VSSC so
      the user can see how far it has reached
    */
    if (fwupdate.node_id != 0) {
	node_status.vendor_specific_status_code = fwupdate.offset / 1024;
	node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    }

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  receive one frame, only called from interrupt context
*/
void DroneCAN_receiveFrame(void)
{
    CanardCANFrame rx_frame = {0};
    while (sys_can_receive(&rx_frame) > 0) {
        canardHandleRxFrame(&canard, &rx_frame, micros64());
    }
}

/*
  Transmits all frames from the TX queue
*/
void DroneCAN_processTxQueue(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = sys_can_transmit(txf);
        if (tx_res == 0) {
            // no space, stop trying
            break;
        }
        // success or error, remove frame
        canardPopTxQueue(&canard);
    }
}

//#pragma GCC optimize("O0")

static void DroneCAN_Startup(void)
{
    canardInit(&canard,
	       canard_memory_pool,              // Raw memory chunk used for dynamic allocation
               sizeof(canard_memory_pool),
	       onTransferReceived,                // Callback, see CanardOnTransferReception
	       shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
	       NULL);

    /*
      if doing fw update get node ID from main firmware via RTC backup
      register
     */
    uint8_t node_id = 0;
    const uint32_t rtc0 = get_rtc_backup_register(0);
    if ((rtc0 & 0xFFFFU) == RTC_BKUP0_FWUPDATE) {
        node_id = rtc0 >> 24;
        uint8_t src_node = (rtc0 >> 16) & 0xFF;
        uint32_t path[2];
        path[0] = get_rtc_backup_register(1);
        path[1] = get_rtc_backup_register(2);
        if (path[0] != 0 && src_node <= 127) {
            // we have update path, can start immediately
            fwupdate.node_id = src_node;
            memset(fwupdate.path, 0, sizeof(fwupdate.path));
            memcpy(fwupdate.path, path, sizeof(path));
        }
    }
    canardSetLocalNodeID(&canard, node_id);

    // initialise low level CAN peripheral hardware
    sys_can_init();

#if 0
    if (fwupdate.node_id != 0) {
        can_print("fwupdate startup");
        can_print(fwupdate.path);
    } else {
        can_print("bootloader startup");
    }
#endif
}

/*
  handle DroneCAN packets. Return true if we are ready to boot the main firmware
 */
bool DroneCAN_update()
{
    static uint64_t next_1hz_service_at;
    static bool done_startup;
    if (!done_startup) {
	done_startup = true;
	DroneCAN_Startup();
    }

    sys_can_disable_IRQ();

    DroneCAN_processTxQueue();

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
	// we're still waiting for a DNA allocation of our node ID
	if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
	    request_DNA();
	}
        sys_can_enable_IRQ();
        return false;
    }

    const uint64_t ts = micros64();

    if (ts >= next_1hz_service_at) {
	next_1hz_service_at += 1000000ULL;
	process1HzTasks(ts);
    }

    if (fwupdate.node_id != 0) {
	send_firmware_read();
    }

    DroneCAN_processTxQueue();

    sys_can_enable_IRQ();

    return DroneCAN_boot_ok();
}

/*
  implementation of memmem() for finding app signature
 */
static void *memmem(const void *haystack, size_t haystacklen, const void *needle, size_t needlelen)
{
    while (haystacklen >= needlelen) {
        char *p = (char *)memchr(haystack, *(const char *)needle, haystacklen-(needlelen-1));
        if (!p) return NULL;
        if (memcmp(p, needle, needlelen) == 0) {
            return p;
        }
        haystack = p+1;
        haystacklen -= (p - (const char *)haystack) + 1;
    }
    return NULL;
}

/*
  see if we are OK to boot
 */
bool DroneCAN_boot_ok(void)
{
    if (fwupdate.node_id != 0) {
        // in fw update
        return false;
    }
    if ((get_rtc_backup_register(0) & 0xFFFFU) == RTC_BKUP0_FWUPDATE) {
        // waiting for firmware update
        node_status.vendor_specific_status_code = FAIL_REASON_IN_UPDATE;
        return false;
    }
    /*
      check application signature
     */
    uint32_t sig[2] = { APP_SIGNATURE_MAGIC1, APP_SIGNATURE_MAGIC2 };
    const uint32_t app_max_len = (128-18)*1024;
    const uint8_t *fw_base = (const uint8_t *)MAIN_FW_START_ADDR;
    struct app_signature *appsig = memmem(fw_base, app_max_len, sig, sizeof(sig));
    if (appsig == NULL || (((uint32_t)appsig) & 3) != 0) {
        node_status.vendor_specific_status_code = FAIL_REASON_NO_APP_SIG;
        return false;
    }
    if (appsig->fwlen > app_max_len) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_LENGTH_APP;
        return false;
    }
    if (memcmp(AM32_MCU, appsig->mcu, strlen(AM32_MCU)) != 0) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_BOARD_ID;
        return false;
    }
    const uint8_t *appsigend = (const uint8_t *)(appsig+1);
    const uint32_t crc1 = crc32(fw_base, (uint32_t)((uint8_t*)appsig - fw_base));
    const uint32_t crc2 = crc32(appsigend, appsig->fwlen - (uint32_t)(appsigend - fw_base));
    if (appsig->crc1 != crc1 ||
        appsig->crc2 != crc2) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_CRC;
        return false;
    }

    if (!have_raw_command) {
        node_status.vendor_specific_status_code = FAIL_REASON_NO_SIGNAL;
        return false;
    }

    node_status.vendor_specific_status_code = CHECK_FW_OK;

    return true;
}

#endif // DRONECAN_SUPPORT
