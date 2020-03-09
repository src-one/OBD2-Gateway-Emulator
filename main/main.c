#include <string.h>
#include <sys/param.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
//#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_eth.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "CAN.h"
#include "CAN_config.h"

#include "fs.c"
#include "fs.h"
#include "obd.c"
#include "obd.h"

static const char *TAG = "MAIN";

#ifndef CONFIG_ESPCAN
#error for this demo you must enable and configure ESPCan in menuconfig
#endif

#ifdef CONFIG_CAN_SPEED_100KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_100KBPS
#endif

#ifdef CONFIG_CAN_SPEED_125KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_125KBPS
#endif

#ifdef CONFIG_CAN_SPEED_250KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_250KBPS
#endif

#ifdef CONFIG_CAN_SPEED_500KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_500KBPS
#endif

#ifdef CONFIG_CAN_SPEED_800KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_800KBPS
#endif

#ifdef CONFIG_CAN_SPEED_1000KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_1000KBPS
#endif

#ifdef CONFIG_CAN_SPEED_USER_KBPS
#define CONFIG_SELECTED_CAN_SPEED CONFIG_CAN_SPEED_USER_KBPS_VAL /* per menuconfig */
#endif

//#define DEBUG true

CAN_device_t CAN_cfg = {
    .speed = CONFIG_SELECTED_CAN_SPEED,      // CAN Node baudrade
    .tx_pin_id = CONFIG_ESP_CAN_TXD_PIN_NUM, // CAN TX pin example menuconfig GPIO_NUM_5
    .rx_pin_id = CONFIG_ESP_CAN_RXD_PIN_NUM, // CAN RX pin example menuconfig GPIO_NUM_35 ( Olimex )
    .rx_queue = NULL,                        // FreeRTOS queue for RX frames
};

// Queue for CAN multi-frame packets
uint8_t can_flow_queue[5][8];

char vehicle_vin[17] = "ESP32OBD2EMULATOR";
unsigned int speed = 0;
float rpm = 0;
float throttle = 0;
int engine_load = 0;
int engine_coolant = 0;
int engine_oil = 0;
float engine_demand_torque = 0;
float engine_actual_torque = 0;
double engine_reference_torque = 0;
double engine_percent_torque = 0;

static EventGroupHandle_t wifi_event_group;

#define WIFI_SSID "ESP32-OBD2"
#define WIFI_PASS "88888888"

#define WIFI_STA_SSID "FuckingAwesomeNet"
#define WIFI_STA_PASS "5bier10schnaps"

//#define WIFI_STA_SSID "reBuy.com guests"
//#define WIFI_STA_PASS "HelloGuest!"

//#define WIFI_STA_SSID "reBuy.com"
//#define WIFI_STA_PASS "reBuy@Schoeneberg!"

#define WIFI_MAXIMUM_RETRY 3

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

CAN_frame_t createOBDResponse(unsigned int mode, unsigned int pid)
{
    CAN_frame_t response;

    response.MsgID = 0x7E8;
    response.FIR.B.DLC = 8;
    response.FIR.B.FF = CAN_frame_std;
    response.FIR.B.RTR = CAN_no_RTR;
    response.data.u8[0] = 2;           // Number of data bytes ()
    response.data.u8[1] = 0x40 + mode; // Mode (+ 0x40)
    response.data.u8[2] = pid;         // PID

    return response;
}

// Define the set of PIDs you wish you ECU to support.  For more information, see:
// https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00
// For this sample, we are only supporting the following PIDs
// PID (HEX)  PID (DEC)  DESCRIPTION
// ---------  ---------  --------------------------
//      0x05         05  Engine Coolant Temperature
//      0x0C         12  Engine RPM
//      0x10         16  MAF Air Flow Rate

// As per the information on bitwise encoded PIDs (https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00)
// Our supported PID value is:
//     PID 0x05 (05) - Engine Coolant Temperature
//     |      PID 0x0C (12) - Engine RPM
//     |      |   PID 0x10 (16) - MAF Air Flow Rate
//     |      |   |
//     V      V   V
// 00001000000100010000000000000000
// Converted to hex, that is the following four byte value
// 0x08110000

// Of course, if you want your ECU simulator to be able to respond to any PID From 0x00 to 0x20, you can just use the following PID Bit mask
// 11111111111111111111111111111111
// Or
// 0xFFFFFFFF

// Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
// we determined above (0x08110000):

//                      0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
//                       |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
//                       |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
//                       |     |     |    0x08 - The first of four bytes of the Supported PIDS value
//                       |     |     |     |    0x11 - The second of four bytes of the Supported PIDS value
//                       |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
//                       |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
//                       |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
//                       |     |     |     |     |      |    |     |
//                       V     V     V     V     V      V    V     V
// byte SupportedPID[8] = {0x06, 0x41, 0x00, 0x08, 0x11, 0x00, 0x00, 0x00};

int sendOBDResponse(CAN_frame_t *response)
{
    int success = CAN_write_frame(response);

#ifdef DEBUG
    printf("Response = %d | (0x%03x) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
           success, response->MsgID,
           response->data.u8[0], response->data.u8[1], response->data.u8[2], response->data.u8[3],
           response->data.u8[4], response->data.u8[5], response->data.u8[6], response->data.u8[7]);
#endif

    return success;
}

void respondToOBD1(uint8_t pid)
{
    #ifdef DEBUG
        printf("Responding to Mode 1 PID 0x%02x\n", pid);
    #endif

    CAN_frame_t response = createOBDResponse(1, pid);

    switch (pid)
    {
        /*
    case 0x00:                      // Supported PIDs
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x00; // Data byte 1
        response.data.u8[4] = 0x18; // Data byte 2
        response.data.u8[5] = 0x80; // Data byte 3
        response.data.u8[6] = 0x00; // Data byte 4
        break;
    */
        /*
    case 0x00:                      // Supported PIDs
        response.data.u8[0] += 6;   // Number of data bytes
        response.data.u8[3] = 0x00; // Data byte 1
        response.data.u8[4] = 0x08; // Data byte 2
        response.data.u8[5] = 0x11; // Data byte 3
        response.data.u8[6] = 0x00; // Data byte 4
        response.data.u8[7] = 0x00; // Data byte 5
        response.data.u8[8] = 0x00; // Data byte 6
        break;
    */
    case 0x00:                      // Supported PIDs 01 - 20
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x18; // Data byte 1
        response.data.u8[4] = 0x1A; // Data byte 2
        response.data.u8[5] = 0x80; // Data byte 3
        response.data.u8[6] = 0x01; // Data byte 4
        break;

    case 0x20:                      // Supported PIDs 21 - 40
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x00; // Data byte 1
        response.data.u8[4] = 0x00; // Data byte 2
        response.data.u8[5] = 0x00; // Data byte 3
        response.data.u8[6] = 0x01; // Data byte 4
        break;

    case 0x40:                      // Supported PIDs 41 - 60
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x00; // Data byte 1
        response.data.u8[4] = 0x00; // Data byte 2
        response.data.u8[5] = 0x00; // Data byte 3
        response.data.u8[6] = 0x11; // Data byte 4
        break;

    case 0x60:                      // Supported PIDs 61 - 80
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0xE0; // Data byte 1
        response.data.u8[4] = 0x00; // Data byte 2
        response.data.u8[5] = 0x00; // Data byte 3
        response.data.u8[6] = 0x01; // Data byte 4
        break;

    case 0x80:                      // Supported PIDs 81 - A0
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x00; // Data byte 1
        response.data.u8[4] = 0x00; // Data byte 2
        response.data.u8[5] = 0x00; // Data byte 3
        response.data.u8[6] = 0x00; // Data byte 4
        break;

    case 0x04:                    // Engine Load
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_04(engine_load, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x05:                    // Engine coolant temperature
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_05(engine_coolant, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x0C:                    // RPM
        response.data.u8[0] += 2; // Number of data bytes
        obdRevConvert_0C(rpm, &response.data.u8[3], &response.data.u8[4], 0, 0);
        break;

    case 0x0D:                    // Speed
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_0D(speed, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x11:                    // Throttle position
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_11(throttle, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x5C:                    // Engine coolant temperature
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_5C(engine_oil, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x61:                    // Driver's demand torque
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_61(engine_demand_torque, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x62:                    // Actual engine torque
        response.data.u8[0] += 1; // Number of data bytes
        obdRevConvert_62(engine_actual_torque, &response.data.u8[3], 0, 0, 0);
        break;

    case 0x63:                    // Engine reference torque
        response.data.u8[0] += 2; // Number of data bytes
        obdRevConvert_63(engine_reference_torque, &response.data.u8[3], &response.data.u8[4], 0, 0);
        break;

    case 0x64:                    // Engine reference torque
        response.data.u8[0] += 5; // Number of data bytes
        obdRevConvert_64(engine_percent_torque, &response.data.u8[3], &response.data.u8[4], &response.data.u8[5], &response.data.u8[6], &response.data.u8[7]);
        break;
    }

    sendOBDResponse(&response);
}

void respondToOBD9(uint8_t pid)
{
    #ifdef DEBUG
        printf("Responding to Mode 9 PID 0x%02x\n", pid);
    #endif

    CAN_frame_t response = createOBDResponse(9, pid);

    switch (pid)
    {

    case 0x00:                      // Supported PIDs [01 - 20]
        response.data.u8[0] += 4;   // Number of data bytes
        response.data.u8[3] = 0x40; // Data byte 1
        response.data.u8[4] = 0x00; // Data byte 2
        response.data.u8[5] = 0x00; // Data byte 3
        response.data.u8[6] = 0x00; // Data byte 4
        sendOBDResponse(&response);
        break;

    case 0x02: // Vehicle Identification Number (VIN)
        // Initiate multi-frame message packet
        response.data.u8[0] = 0x10;           // FF (First Frame, ISO_15765-2)
        response.data.u8[1] = 0x14;           // Length (20 bytes)
        response.data.u8[2] = 0x49;           // Mode (+ 0x40)
        response.data.u8[3] = 0x02;           // PID
        response.data.u8[4] = 0x01;           // Data byte 1
        response.data.u8[5] = vehicle_vin[0]; // Data byte 2
        response.data.u8[6] = vehicle_vin[1]; // Data byte 3
        response.data.u8[7] = vehicle_vin[2]; // Data byte 4
        sendOBDResponse(&response);

        // Clear flow control queue
        // memset(can_flow_queue, 0, 40);

        // Fill flow control queue
        // Part 1
        can_flow_queue[0][0] = 0x21;           // CF (Consecutive Frame, ISO_15765-2), sequence number
        can_flow_queue[0][1] = vehicle_vin[3]; // Data byte 1
        can_flow_queue[0][2] = vehicle_vin[4]; // Data byte 2
        can_flow_queue[0][3] = vehicle_vin[5]; // Data byte 3
        can_flow_queue[0][4] = vehicle_vin[6]; // Data byte 4
        can_flow_queue[0][5] = vehicle_vin[7]; // Data byte 5
        can_flow_queue[0][6] = vehicle_vin[8]; // Data byte 6
        can_flow_queue[0][7] = vehicle_vin[9]; // Data byte 7
        // Part 2
        can_flow_queue[1][0] = 0x22;            // CF
        can_flow_queue[1][1] = vehicle_vin[10]; // Data byte 1
        can_flow_queue[1][2] = vehicle_vin[11]; // Data byte 2
        can_flow_queue[1][3] = vehicle_vin[12]; // Data byte 3
        can_flow_queue[1][4] = vehicle_vin[13]; // Data byte 4
        can_flow_queue[1][5] = vehicle_vin[14]; // Data byte 5
        can_flow_queue[1][6] = vehicle_vin[15]; // Data byte 6
        can_flow_queue[1][7] = vehicle_vin[16]; // Data byte 7

        break;
    }
}

void task_CAN(void *pvParameters)
{
    (void)pvParameters;

    //frame buffer
    CAN_frame_t __RX_frame;

    //create CAN RX Queue
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));

    //start CAN Module
    CAN_init();
    printf("CAN initialized...\n");

    while (1)
    {
        //receive next CAN frame from queue
        if (xQueueReceive(CAN_cfg.rx_queue, &__RX_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
        {
            #ifdef DEBUG
                printf("\nFrame from : 0x%08x, DLC %d, RTR %d, FF %d \n", __RX_frame.MsgID, __RX_frame.FIR.B.DLC,
                    __RX_frame.FIR.B.RTR, __RX_frame.FIR.B.FF);
                printf("D0: 0x%02x, ", __RX_frame.data.u8[0]);
                printf("D1: 0x%02x, ", __RX_frame.data.u8[1]);
                printf("D2: 0x%02x, ", __RX_frame.data.u8[2]);
                printf("D3: 0x%02x, ", __RX_frame.data.u8[3]);
                printf("D4: 0x%02x, ", __RX_frame.data.u8[4]);
                printf("D5: 0x%02x, ", __RX_frame.data.u8[5]);
                printf("D6: 0x%02x, ", __RX_frame.data.u8[6]);
                printf("D7: 0x%02x\n", __RX_frame.data.u8[7]);
                printf("==============================================================================\n");
            #endif

            // Check if frame is OBD query
            if (__RX_frame.MsgID == 0x7df)
            {
                #ifdef DEBUG
                    printf("OBD QUERY !\n");
                #endif

                switch (__RX_frame.data.u8[1])
                {       // Mode
                case 1: // Show current data
                    respondToOBD1(__RX_frame.data.u8[2]);
                    break;
                case 9: // Vehicle information
                    respondToOBD9(__RX_frame.data.u8[2]);
                    break;
                default:
                    //#ifdef DEBUG
                        printf("Unsupported mode %d !\n", __RX_frame.data.u8[1]);
                    //#endif
                }
            }
            else if (__RX_frame.MsgID == 0x7e0)
            { // Check if frame is addressed to the ECU (us)
                #ifdef DEBUG
                    printf("ECU MSG !\n");
                #endif

                if (__RX_frame.data.u8[0] == 0x30)
                { // Flow control frame (continue)
                    CAN_frame_t response = createOBDResponse(0, 0);

                    for (int i = 0; i < 5; i++)
                    {
                        if (can_flow_queue[i][0] == 0)
                        {
                            continue;
                        }

                        for (int j = 0; j < 8; j++)
                        {
                            response.data.u8[j] = can_flow_queue[i][j];
                        }
                        sendOBDResponse(&response);
                    }

                    // Clear flow control queue
                    memset(can_flow_queue, 0, 40);
                }
            }
        }
    }
}

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context
{
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)
/*
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html"))
    {
        type = "text/html";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".js"))
    {
        type = "application/javascript";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".css"))
    {
        type = "text/css";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".png"))
    {
        type = "image/png";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".ico"))
    {
        type = "image/x-icon";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".svg"))
    {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}
*/
/* HTTP GET handler for downloading files */
esp_err_t file_get_handler(httpd_req_t *req)
{
    const char *filepath_prefix = "/spiflash/";
    char *filename = NULL;
    size_t filename_len = httpd_req_get_url_query_len(req);

    const char *ctx = (const char *)req->user_ctx;

    FILE *file = NULL;

    if (ctx != NULL)
    {

        ESP_LOGI(TAG, "Use context: %s", (char *)req->user_ctx);

        file = fopen(ctx, "r");

        ESP_LOGI(TAG, "Reading file : %s", ctx);
    }
    else
    {
        if (filename_len == 0)
        {
            const char *resp_str = "Please specify a filename. eg. file?somefile.txt";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            return ESP_OK;
        }
        /*
        filename = malloc(strlen(filepath_prefix) + filename_len + 1);
        strncpy(filename, filepath_prefix, strlen(filepath_prefix));

        httpd_req_get_url_query_str(req, filename + strlen(filepath_prefix), filename_len + 1);

        file = fopen(filename, "r");
        free(filename);
        */
        ESP_LOGI(TAG, "Reading file : %s", filename + strlen(filepath_prefix));
    }

    if (file == NULL)
    {
        const char *resp_str = "File doesn't exist";

        ESP_LOGI(TAG, "File doesn't exist: %s", filename);

        httpd_resp_send(req, resp_str, strlen(resp_str));

        return ESP_OK;
    }

    /* Read file in chunks (relaxes any constraint due to large file sizes)
     * and send HTTP response in chunked encoding */
    char chunk[1024];
    size_t chunksize;

    do
    {
        chunksize = fread(chunk, 1, sizeof(chunk), file);
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
        {
            fclose(file);
            return ESP_FAIL;
        }
    } while (chunksize != 0);

    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);

    return ESP_OK;
}

/*
static esp_err_t rest_common_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/')
    {
        strlcat(filepath, "/index.html", sizeof(filepath));
    }
    else
    {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1)
    {
        ESP_LOGE(REST_TAG, "Failed to open file : %s", filepath);

        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do
    {
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1)
        {
            ESP_LOGE(REST_TAG, "Failed to read file : %s", filepath);
        }
        else if (read_bytes > 0)
        {
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
            {
                close(fd);
                ESP_LOGE(REST_TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    close(fd);
    ESP_LOGI(REST_TAG, "File sending complete");

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
*/

esp_err_t api_get_vehicle_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;

    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1)
    {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;

    if (buf_len > 1)
    {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];

            if (httpd_query_key_value(buf, "value", param, sizeof(param)) == ESP_OK)
            {
                ESP_LOGI(TAG, "Found URL query parameter => value = %s", param);

                if (strcmp(param, "speed") == 0)
                {
                    ESP_LOGI(TAG, "speed = %d", speed);
                }
                else if (strcmp(param, "rpm") == 0)
                {
                    ESP_LOGI(TAG, "rpm = %f", rpm);
                }
                else if (strcmp(param, "throttle") == 0)
                {
                    ESP_LOGI(TAG, "throttle = %f", throttle);
                }
                else if (strcmp(param, "engine_load") == 0)
                {
                    ESP_LOGI(TAG, "engine_load = %d", engine_load);
                }
                else if (strcmp(param, "engine_demand_torque") == 0)
                {
                    ESP_LOGI(TAG, "engine_demand_torque = %f", engine_demand_torque);
                }
                else if (strcmp(param, "engine_actual_torque") == 0)
                {
                    ESP_LOGI(TAG, "engine_actual_torque = %f", engine_actual_torque);
                }
                else if (strcmp(param, "engine_reference_torque") == 0)
                {
                    ESP_LOGI(TAG, "engine_reference_torque = %f", engine_reference_torque);
                }
                else if (strcmp(param, "engine_percent_torque") == 0)
                {
                    ESP_LOGI(TAG, "engine_percent_torque = %f", engine_percent_torque);
                }
                else if (strcmp(param, "engine_coolant") == 0)
                {
                    ESP_LOGI(TAG, "engine_coolant = %d", engine_coolant);
                }
                else if (strcmp(param, "engine_oil") == 0)
                {
                    ESP_LOGI(TAG, "engine_oil = %d", engine_oil);
                }
                else if (strcmp(param, "vin") == 0)
                {
                    ESP_LOGI(TAG, "vehicle_vin = %s", vehicle_vin);
                }
            }
        }

        free(buf);
    }

    const char *resp_str = (const char *)req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0)
    {
        ESP_LOGI(TAG, "Request headers lost");
    }

    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

esp_err_t api_patch_vehicle_handler(httpd_req_t *req)
{
    char name[50] = "";

    int total_len = req->content_len;
    int cur_len = 0;

    char *_buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;

    if (total_len >= SCRATCH_BUFSIZE)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }

    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, _buf + cur_len, total_len);

        if (received <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }

    _buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(_buf);

    if (root == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();

        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
        }

        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON error");
        return ESP_FAIL;
    }

    char *_name = cJSON_GetObjectItem(root, "name")->valuestring;
    strncpy(name, _name, 50);

    cJSON *value = cJSON_GetObjectItem(root, "value");

    ESP_LOGI(TAG, "Set ECU value: name = %s, value = %.2f", name, value->valuedouble);

    cJSON_Delete(root);

    if (name != NULL && value != NULL)
    {
        if (cJSON_IsNumber(value))
        {
            printf("Received %s = %.2f\n", name, value->valuedouble);
        }
        else
        {
            printf("Received %s = %s\n", name, value->valuestring);
        }

        if (strcmp(name, "speed") == 0 && cJSON_IsNumber(value))
        {
            speed = value->valueint;
            ESP_LOGI(TAG, "Set speed => %d", speed);
        }
        else if (strcmp(name, "rpm") == 0 && cJSON_IsNumber(value))
        {
            rpm = value->valuedouble;
            ESP_LOGI(TAG, "Set rpm => %f", rpm);
        }
        else if (strcmp(name, "throttle") == 0 && cJSON_IsNumber(value))
        {
            throttle = value->valuedouble;
            ESP_LOGI(TAG, "Set throttle => %f", throttle);
        }
        else if (strcmp(name, "engine_load") == 0 && cJSON_IsNumber(value))
        {
            engine_load = value->valueint;
            ESP_LOGI(TAG, "Set engine_load => %d", engine_load);
        }
        else if (strcmp(name, "engine_demand_torque") == 0 && cJSON_IsNumber(value))
        {
            engine_demand_torque = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_demand_torque => %f", engine_demand_torque);
        }
        else if (strcmp(name, "engine_actual_torque") == 0 && cJSON_IsNumber(value))
        {
            engine_actual_torque = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_actual_torque => %f", engine_actual_torque);
        }
        else if (strcmp(name, "engine_reference_torque") == 0 && cJSON_IsNumber(value))
        {
            engine_reference_torque = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_reference_torque => %f", engine_reference_torque);
        }
        else if (strcmp(name, "engine_reference_torque") == 0 && cJSON_IsNumber(value))
        {
            engine_percent_torque = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_percent_torque => %f", engine_percent_torque);
        }
        else if (strcmp(name, "engine_coolant") == 0 && cJSON_IsNumber(value))
        {
            engine_coolant = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_coolant => %d", engine_coolant);
        }
        else if (strcmp(name, "engine_oil") == 0 && cJSON_IsNumber(value))
        {
            engine_oil = value->valuedouble;
            ESP_LOGI(TAG, "Set engine_oil => %d", engine_oil);
        }
        else if (strcmp(name, "vin") == 0 && cJSON_IsString(value))
        {
            strncpy(vehicle_vin, value->valuestring, 17);
            ESP_LOGI(TAG, "Set vin => %s", vehicle_vin);
        }
    }
    else
    {
        printf("Invalid data received!\n");
    }

    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

esp_err_t start_webserver(const char *base_path)
{

    if (!base_path)
    {
        ESP_LOGE(TAG, "wrong base path");
        return ESP_FAIL;
    }

    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));

    if (!rest_context)
    {
        ESP_LOGE(TAG, "No memory for rest context");
        return ESP_FAIL;
    }

    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");

        const httpd_uri_t file_serve = {
            .uri = "/assets",
            .method = HTTP_GET,
            .handler = file_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &file_serve);

        const httpd_uri_t http_get_root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = file_get_handler,
            .user_ctx = "/spiflash/index.html"};
        httpd_register_uri_handler(server, &http_get_root);

        const httpd_uri_t http_get_index = {
            .uri = "/index.html",
            .method = HTTP_GET,
            .handler = file_get_handler,
            .user_ctx = "/spiflash/index.html"};
        httpd_register_uri_handler(server, &http_get_index);

        const httpd_uri_t http_get_main_css = {
            .uri = "/main.css",
            .method = HTTP_GET,
            .handler = file_get_handler,
            .user_ctx = "/spiflash/main.css"};
        httpd_register_uri_handler(server, &http_get_main_css);

        const httpd_uri_t http_get_main_js = {
            .uri = "/main.js",
            .method = HTTP_GET,
            .handler = file_get_handler,
            .user_ctx = "/spiflash/main.js"};
        httpd_register_uri_handler(server, &http_get_main_js);

        const httpd_uri_t api_get_vehicle_uri = {
            .uri = "/api/vehicle",
            .method = HTTP_GET,
            .handler = api_get_vehicle_handler,
            .user_ctx = "value set!"};
        httpd_register_uri_handler(server, &api_get_vehicle_uri);

        const httpd_uri_t api_patch_vehicle_uri = {
            .uri = "/api/vehicle",
            .method = HTTP_PATCH,
            .handler = api_patch_vehicle_handler,
            .user_ctx = rest_context};
        httpd_register_uri_handler(server, &api_patch_vehicle_uri);

        return ESP_OK;
    }

    free(rest_context);
    ESP_LOGI(TAG, "Error starting server!");

    return ESP_FAIL;
}

static int s_retry_num = 0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }

        ESP_LOGI(TAG, "connect to the AP fail\n");
        break;
    }

    default:
        break;
    }

    return ESP_OK;
}

void wifi_init_softap()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_sta_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASS}};

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK}};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_STA_SSID, WIFI_STA_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_STA_SSID, WIFI_STA_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    vEventGroupDelete(wifi_event_group);
}

void app_main()
{
    // wait for IDF logs to end
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("CAN RXD PIN NUM: %d\n", CONFIG_ESP_CAN_RXD_PIN_NUM);
    printf("CAN TXD PIN NUM: %d\n", CONFIG_ESP_CAN_TXD_PIN_NUM);
    printf("CAN SPEED      : %d KBit/s\n", CONFIG_SELECTED_CAN_SPEED);
#ifdef CONFIG_CAN_SPEED_USER_KBPS
    printf("kBit/s setting was done by User\n");
#endif

    //Create CAN receive task
    xTaskCreate(&task_CAN, "CAN", 2048, NULL, 5, NULL);

    ///////////////// WIFI

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("Initializing WIFI...\n");
    wifi_init_softap();

    ///////////////// HTTP

    printf("Initializing HTTP server...\n");

    start_webserver("/spiflash");
    //start_rest_server("/image");

    ////////////////// FAT

    esp_vfs_fat_mount_config_t mountConfig;
    wl_handle_t m_wl_handle;
    mountConfig.max_files = 4;
    mountConfig.format_if_mount_failed = false;

    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount("/spiflash", "storage", &mountConfig, &m_wl_handle));

    ESP_ERROR_CHECK(dumpDir("/spiflash"));
}
