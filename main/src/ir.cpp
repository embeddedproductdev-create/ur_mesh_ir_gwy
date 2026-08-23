#include "esp_log.h"
#include <stdbool.h>

#include "IRac.h"
#include <IRutils.h>
#include <IRremoteESP8266.h>

#include <main.h>
#include <ir.h>
#include <lte.h>
#include <flash.h>
#include <ble_new.h>
#include <led.h>
#include <json_maker.h>
#include <cJSON.h>

extern "C" {
#include <temperature_sensor.h>
}

#define RAW_FREQ 41

const char *RAW_IR_PROTOCOL = "RAW";
const char *DAIKIN_IR_PROTOCOL = "DAIKIN280";
const char *DAIKIN200_IR_PROTOCOL = "DAIKIN200";
const char *DAIKIN216_IR_PROTOCOL = "DAIKIN216";
const char *DAIKIN2_IR_PROTOCOL = "DAIKIN2";
const char *DAIKIN160_IR_PROTOCOL = "DAIKIN160";
const char *DAIKIN176_IR_PROTOCOL = "DAIKIN176";
const char *DAIKIN64_IR_PROTOCOL = "DAIKIN64";
const char *DAIKIN152_IR_PROTOCOL = "DAIKIN152";
const char *DAIKIN128_IR_PROTOCOL = "DAIKIN128";
const char *HITACHI_AC296_IR_PROTOCOL = "HITACHI_AC296";
const char *HITACHI_AC_IR_PROTOCOL = "HITACHI_AC";
const char *HITACHI_AC1_IR_PROTOCOL = "HITACHI_AC1";
const char *HITACHI_AC424_IR_PROTOCOL = "HITACHI_AC424";
const char *HITACHI_AC344_IR_PROTOCOL = "HITACHI_AC344";
const char *HITACHI_AC264_IR_PROTOCOL = "HITACHI_AC264";
const char *VOLTAS_IR_PROTOCOL = "VOLTAS";
const char *SAMSUNG_AC_IR_PROTOCOL = "SAMSUNG_AC";
const char *HAIER_AC_IR_PROTOCOL = "HAIER_AC";
const char *HAIER_AC176_IR_PROTOCOL = "HAIER_AC176";
const char *HAIER_AC160_IR_PROTOCOL = "HAIER_AC160";
const char *CARRIER_AC64_IR_PROTOCOL = "CARRIER_AC64";
const char *LG2_IR_PROTOCOL = "LG2";
const char *LG_IR_PROTOCOL = "LG";
const char *TOSHIBA_AC_IR_PROTOCOL = "TOSHIBA_AC";
const char *MITSUBISHI112_IR_PROTOCOL = "MITSUBISHI112";
const char *MITSUBISHI136_IR_PROTOCOL = "MITSUBISHI136";
const char *MITSUBISHI_AC_IR_PROTOCOL = "MITSUBISHI_AC";
const char *MITSUBISHI_HEAVY_88_IR_PROTOCOL = "MITSUBISHI_HEAVY_88";
const char *MITSUBISHI_HEAVY_152_IR_PROTOCOL = "MITSUBISHI_HEAVY_152";
const char *COOLIX_IR_PROTOCOL = "COOLIX";
const char *FUJITSU_AC_IR_PROTOCOL = "FUJITSU_AC";
const char *UNKNOWN_IR_PROTOCOL = "UNKNOWN";
const char *UNUSED_IR_PROTOCOL = "UNUSED";
const char *INVALID_IR_PROTOCOL = "INVALID";

const char *TEACHING_MODE_SEQUENCE[] = {
    "Power - Off",                   // 0
    "Temperature - 18 | Power - On", // 1
    "Temperature - 19 | Power - On", // 2
    "Temperature - 20 | Power - On", // 3
    "Temperature - 21 | Power - On", // 4
    "Temperature - 22 | Power - On", // 5
    "Temperature - 23 | Power - On", // 6
    "Temperature - 24 | Power - On", // 7
    "Temperature - 25 | Power - On", // 8
    "Temperature - 26 | Power - On", // 9
    "Temperature - 27 | Power - On", // 10
    "Temperature - 28 | Power - On", // 11
    "Temperature - 29 | Power - On", // 12
    "Temperature - 30 | Power - On", // 13
    "Temperature - 31 | Power - On", // 14
    "Temperature - 32 | Power - On"  // 15
};

bool teaching_mode_cmd_recvd_array[MAX_CMDS_IN_TEACHING_MODE];

/*IR Receiver Initializations*/
IRrecv irrecv(IR_RECV_GPIO, IR_RECV_BUFFER_LEN, KTIMEOUT, SAVE_BUFFER_FLAG);
decode_results results;
const uint8_t kTolerancePercentage = 25;

manual_control ac_manual_control_t;
teaching_mode teaching_mode_t;

/*IR Transmitter Initializations*/

/*AC Class Objects*/
IRCarrierAc64 ac_carrier64(IR_TRAN_GPIO);

IRDaikinESP ac_daikin280(IR_TRAN_GPIO);
IRDaikin216 ac_daikin216(IR_TRAN_GPIO);
IRDaikin2 ac_daikin2(IR_TRAN_GPIO);
IRDaikin160 ac_daikin160(IR_TRAN_GPIO);
IRDaikin176 ac_daikin176(IR_TRAN_GPIO);
IRDaikin200 ac_daikin200(IR_TRAN_GPIO);
IRDaikin64 ac_daikinac64(IR_TRAN_GPIO);
IRDaikin152 ac_daikin152(IR_TRAN_GPIO);
IRDaikin128 ac_daikin128(IR_TRAN_GPIO);

IRHaierAC ac_haier(IR_TRAN_GPIO);
IRHaierAC176 ac_haier176(IR_TRAN_GPIO);
IRHaierAC160 ac_haier160(IR_TRAN_GPIO);

IRHitachiAc ac_hitachi224(IR_TRAN_GPIO);
IRHitachiAc1 ac_hitachi104(IR_TRAN_GPIO);
IRHitachiAc424 ac_hitachi424(IR_TRAN_GPIO);
IRHitachiAc344 ac_hitachi344(IR_TRAN_GPIO);
IRHitachiAc264 ac_hitachi264(IR_TRAN_GPIO);
IRHitachiAc296 ac_hitachi296(IR_TRAN_GPIO);

IRLgAc ac_lg(IR_TRAN_GPIO);

IRMitsubishi112 ac_mitsubishi112(IR_TRAN_GPIO);
IRMitsubishi136 ac_mitsubishi136(IR_TRAN_GPIO);
IRMitsubishiAC ac_mitsubishi144(IR_TRAN_GPIO);
IRMitsubishiHeavy88Ac ac_mitsubishi88(IR_TRAN_GPIO);
IRMitsubishiHeavy152Ac ac_mitsubishi152(IR_TRAN_GPIO);

IRSamsungAc ac_samsung(IR_TRAN_GPIO);

IRToshibaAC ac_toshiba(IR_TRAN_GPIO);

IRVoltas ac_voltas(IR_TRAN_GPIO);

IRCoolixAC ac_coolix(IR_TRAN_GPIO);

IRFujitsuAC ac_fujitsu(IR_TRAN_GPIO, ARRAH2E);

IRsend ac_custom(IR_TRAN_GPIO);
IRsend ac_general(IR_TRAN_GPIO);



/**
 * @brief Function that initializes the ir Transmitter setup.
 * Calls the begin function of all AC class objects
 */
void ir_tran_setup()
{
    ac_daikin216.begin();
    ac_daikin280.begin();
    ac_daikin2.begin();
    ac_daikin160.begin();
    ac_daikin176.begin();
    ac_daikin200.begin();
    ac_daikinac64.begin();
    ac_daikin152.begin();
    ac_daikin128.begin();
    ac_hitachi296.begin();
    ac_hitachi224.begin();
    ac_hitachi104.begin();
    ac_hitachi424.begin();
    ac_hitachi344.begin();
    ac_hitachi264.begin();
    ac_voltas.begin();
    ac_samsung.begin();
    ac_haier.begin();
    ac_haier176.begin();
    ac_haier160.begin();
    ac_lg.begin();
    ac_toshiba.begin();
    ac_carrier64.begin();
    ac_mitsubishi112.begin();
    ac_mitsubishi136.begin();
    ac_mitsubishi144.begin();
    ac_mitsubishi88.begin();
    ac_mitsubishi152.begin();
    ac_coolix.begin();
    ac_fujitsu.begin();
    ac_custom.begin();
}

/**
 * @brief Function that transmits the ir command to control AC
 *
 */
void ir_transmit()
{
    irrecv.pause();
    ESP_LOGW("IR_TX", "TRANSMIT CALLED: power=%d temp=%d fanspeed=%d mode_num=%d mode_str=%s protocol=%d",
        last_command.power,
        last_command.temperature,
        last_command.fanspeed,
        last_command.mode_num,
        last_command.mode_str,
        ir_protocol_num);
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_set_state(LED_STATE_SENDING_IR_COMMAND);

    // Boost priority to prevent task preemption during IR bit-bang transmission
    UBaseType_t prev_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    switch (ir_protocol_num)
    {
    case CARRIER_AC64:
        ac_carrier64.setPower(last_command.power);
        ac_carrier64.setTemp(last_command.temperature);
        if (last_command.swingv)
            last_command.swingv = kDaikinSwingOn;
        ac_carrier64.setSwingV(last_command.swingv);
        ac_carrier64.setFan(ac_carrier64.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_carrier64.setOnTimer(last_command.ontimer * 60);
        ac_carrier64.setOffTimer(last_command.offtimer * 60);
        ac_carrier64.setMode(ac_carrier64.convertMode((stdAc::opmode_t)last_command.mode_num));   
        ac_carrier64.send();    // ~100ms of uninterrupted bit-bang timing
        break;

    case DAIKIN:
        ESP_LOGW("IR_TX", "DAIKIN280 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin280.setPower(last_command.power);
        ac_daikin280.setTemp(last_command.temperature);
        if (last_command.swingh)
            last_command.swingh = kDaikinSwingOn;
        ac_daikin280.setSwingHorizontal(last_command.swingh);
        if (last_command.swingv)
            last_command.swingv = kDaikinSwingOn;
        ac_daikin280.setSwingVertical(last_command.swingv);
        ac_daikin280.setFan(ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin280.enableOffTimer(last_command.offtimer);
        ac_daikin280.enableOnTimer(last_command.ontimer);
        ac_daikin280.setMode(ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));   
        ac_daikin280.send();
        break;

    case DAIKIN200:
        ESP_LOGW("IR_TX", "DAIKIN200 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin200.setPower(last_command.power);
        ac_daikin200.setSwingHorizontal(last_command.swingh);
        ac_daikin200.setFan(last_command.fanspeed);
        ac_daikin200.setTemp(last_command.temperature);
        ac_daikin200.setMode(ac_daikin200.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin200.send();
        break;

    case DAIKIN216:
        ESP_LOGW("IR_TX", "DAIKIN216 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin216.setPower(last_command.power);
        ac_daikin216.setTemp(last_command.temperature);
        if (last_command.swingh)
            last_command.swingh = kDaikinSwingOn;
        ac_daikin216.setSwingHorizontal(last_command.swingh);
        if (last_command.swingv)
            last_command.swingv = kDaikinSwingOn;
        ac_daikin216.setSwingVertical(last_command.swingv);
        ac_daikin216.setFan(ac_daikin216.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin216.setMode(ac_daikin216.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin216.send();
        break;

    case DAIKIN2:
        ESP_LOGW("IR_TX", "DAIKIN2 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin2.setPower(last_command.power);
        ac_daikin2.setTemp(last_command.temperature);
        if (last_command.swingh)
            ac_daikin2.setSwingHorizontal(kDaikin2SwingHAuto);
        else
            ac_daikin2.setSwingHorizontal(kDaikin2SwingHOff);
        if (last_command.swingv)
            ac_daikin2.setSwingVertical(kDaikin2SwingVAuto);
        else
            ac_daikin2.setSwingVertical(kDaikin2SwingVOff);
        ac_daikin2.setFan(ac_daikin2.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin2.enableOffTimer(last_command.offtimer);
        ac_daikin2.enableOnTimer(last_command.ontimer);
        ac_daikin2.setMode(ac_daikin2.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin2.send();
        break;

    case DAIKIN160:
        ESP_LOGW("IR_TX", "DAIKIN160 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin160.setPower(last_command.power);
        ac_daikin160.setTemp(last_command.temperature);
        if (last_command.swingv)
            ac_daikin160.setSwingVertical(kDaikin160SwingVAuto);
        ac_daikin160.setFan(ac_daikin160.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin160.setMode(ac_daikin160.convertMode((stdAc::opmode_t)last_command.mode_num)); 
        ac_daikin160.send();
        break;

    case DAIKIN176:
        ESP_LOGW("IR_TX", "DAIKIN176 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin176.setPower(last_command.power);
        ac_daikin176.setTemp(last_command.temperature);
        if (last_command.swingh)
            ac_daikin176.setSwingHorizontal(kDaikin176SwingHAuto);
        else
            ac_daikin176.setSwingHorizontal(kDaikin176SwingHOff);
        ac_daikin176.setFan(ac_daikin176.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin176.setMode(ac_daikin176.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin176.send();
        break;

    case DAIKIN64:
        ESP_LOGW("IR_TX", "DAIKIN64 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikinac64.setPowerToggle(last_command.power);
        ac_daikinac64.setTemp(last_command.temperature);
        ac_daikinac64.setSwingVertical(last_command.swingv);
        ac_daikinac64.setFan(ac_daikinac64.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikinac64.setOnTime(last_command.offtimer);
        ac_daikinac64.setOffTime(last_command.ontimer);
        ac_daikinac64.setMode(ac_daikinac64.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikinac64.send();
        break;

    case DAIKIN152:
        ESP_LOGW("IR_TX", "DAIKIN152 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin152.setPower(last_command.power);
        ac_daikin152.setTemp(last_command.temperature);
        ac_daikin152.setSwingV(last_command.swingv);
        ac_daikin152.setFan(ac_daikin152.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin152.setMode(ac_daikin152.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin152.send();
        break;

    case DAIKIN128:
        ESP_LOGW("IR_TX", "DAIKIN128 sending: convertFan=%d convertMode=%d",
        ac_daikin280.convertFan((stdAc::fanspeed_t)last_command.fanspeed),
        ac_daikin280.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_daikin128.setPowerToggle(last_command.power);
        ac_daikin128.setTemp(last_command.temperature);
        ac_daikin128.setSwingVertical(last_command.swingv);
        ac_daikin128.setFan(ac_daikin128.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_daikin128.setOffTimer(last_command.offtimer);
        ac_daikin128.setOnTimer(last_command.ontimer);
        ac_daikin128.setMode(ac_daikin128.convertMode((stdAc::opmode_t)last_command.mode_num)); 
        ac_daikin128.send();
        break;

    case HAIER_AC:
        ac_haier.setTemp(last_command.temperature);
        ac_haier.setSwingV(last_command.swingv);
        ac_haier.setFan(ac_haier.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_haier.setOffTimer(last_command.offtimer);
        ac_haier.setOnTimer(last_command.ontimer);
        ac_haier.setMode(ac_haier.convertMode((stdAc::opmode_t)last_command.mode_num)); 
        ac_haier.send();
        break;

    case HAIER_AC176:
        ac_haier176.setPower(last_command.power);
        ac_haier176.setTemp(last_command.temperature);
        ac_haier176.setSwingH(last_command.swingh);
        ac_haier176.setSwingV(last_command.swingv);
        ac_haier176.setFan(ac_haier176.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_haier176.setOffTimer(last_command.offtimer);
        ac_haier176.setOnTimer(last_command.ontimer);
        ac_haier176.setMode(ac_haier176.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_haier176.send();
        break;

    case HAIER_AC160:
        ac_haier160.setPower(last_command.power);
        ac_haier160.setTemp(last_command.temperature);
        ac_haier160.setSwingV(last_command.swingv);
        ac_haier160.setFan(ac_haier160.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_haier160.setOffTimer(last_command.offtimer);
        ac_haier160.setOnTimer(last_command.ontimer);
        ac_haier160.setMode(ac_haier160.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_haier160.send();
        break;

    case HITACHI_AC296:
        ac_hitachi296.setPower(last_command.power);
        ac_hitachi296.setTemp(last_command.temperature);
        ac_hitachi296.setFan(ac_hitachi296.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_hitachi296.setMode(ac_hitachi296.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_hitachi296.send();
        break;

    case HITACHI_AC:
        ac_hitachi224.setPower(last_command.power);
        ac_hitachi224.setTemp(last_command.temperature);
        ac_hitachi224.setFan(ac_hitachi224.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_hitachi224.setSwingHorizontal(last_command.swingh);
        ac_hitachi224.setSwingVertical(last_command.swingv);
        ac_hitachi224.setMode(ac_hitachi224.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_hitachi224.send();
        break;

    case HITACHI_AC1:
        ac_hitachi104.setPower(last_command.power);
        ac_hitachi104.setTemp(last_command.temperature);
        ac_hitachi104.setFan(ac_hitachi104.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_hitachi104.setSwingH(last_command.swingh);
        ac_hitachi104.setSwingV(last_command.swingv);
        ac_hitachi104.setOffTimer(last_command.offtimer);
        ac_hitachi104.setOnTimer(last_command.ontimer);
        ac_hitachi104.setMode(ac_hitachi104.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_hitachi104.send();
        break;

    case HITACHI_AC424:
        ac_hitachi424.setPower(last_command.power);
        ac_hitachi424.setTemp(last_command.temperature);
        ac_hitachi424.setFan(ac_hitachi424.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_hitachi424.setSwingVToggle(last_command.swingv);
        ac_hitachi424.setMode(ac_hitachi424.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_hitachi424.send();
        break;

    case HITACHI_AC344:
        ac_hitachi344.setSwingH(last_command.swingh);
        ac_hitachi344.setSwingV(last_command.swingv);
        ac_hitachi344.send();
        break;

    case HITACHI_AC264:
        ac_hitachi264.setFan(ac_hitachi264.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_hitachi264.send();
        break;

    case LG2:
    case LG:
        ac_lg.setPower(last_command.power);
        ac_lg.setTemp(last_command.temperature);
        ac_lg.setSwingH(last_command.swingh);
        ac_lg.setSwingV(last_command.swingv);
        ac_lg.setFan(ac_lg.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_lg.setMode(ac_lg.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_lg.send();
        break;

    case MITSUBISHI112:
        ac_mitsubishi112.setPower(last_command.power);
        ac_mitsubishi112.setTemp(last_command.temperature);
        ac_mitsubishi112.setSwingH(last_command.swingh);
        ac_mitsubishi112.setSwingV(last_command.swingv);
        ac_mitsubishi112.setFan(ac_mitsubishi112.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_mitsubishi112.setMode(ac_mitsubishi112.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_mitsubishi112.send();
        break;

    case MITSUBISHI136:
        ac_mitsubishi136.setPower(last_command.power);
        ac_mitsubishi136.setTemp(last_command.temperature);
        if (last_command.swingv)
        {
            ac_mitsubishi136.setSwingV(kMitsubishi136SwingVAuto);
        }
        ac_mitsubishi136.setFan(ac_mitsubishi136.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_mitsubishi136.setMode(ac_mitsubishi136.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_mitsubishi136.send();
        break;

    case MITSUBISHI_AC:
        ac_mitsubishi144.setPower(last_command.power);
        ac_mitsubishi144.setTemp(last_command.temperature);
        ac_mitsubishi144.setFan(ac_mitsubishi144.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_mitsubishi144.setMode(ac_mitsubishi144.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_mitsubishi144.send();
        break;

    case MITSUBISHI_HEAVY_88:
        ac_mitsubishi88.setPower(last_command.power);
        ac_mitsubishi88.setTemp(last_command.temperature);
        ac_mitsubishi88.setFan(ac_mitsubishi88.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        if (last_command.swingh)
            ac_mitsubishi88.setSwingHorizontal(kMitsubishiHeavy88SwingHAuto);
        else
            ac_mitsubishi88.setSwingHorizontal(kMitsubishiHeavy88SwingHOff);
        if (last_command.swingv)
            ac_mitsubishi88.setSwingVertical(kMitsubishiHeavy88SwingVAuto);
        else
            ac_mitsubishi88.setSwingVertical(kMitsubishiHeavy88SwingVOff);
        ac_mitsubishi88.setMode(ac_mitsubishi88.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_mitsubishi88.send();
        break;

    case MITSUBISHI_HEAVY_152:
        ac_mitsubishi152.setPower(last_command.power);
        ac_mitsubishi152.setTemp(last_command.temperature);
        ac_mitsubishi152.setFan(ac_mitsubishi152.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        if (last_command.swingh)
            ac_mitsubishi152.setSwingHorizontal(kMitsubishiHeavy88SwingHAuto);
        else
            ac_mitsubishi152.setSwingHorizontal(kMitsubishiHeavy88SwingHOff);
        if (last_command.swingv)
            ac_mitsubishi88.setSwingVertical(kMitsubishiHeavy88SwingVAuto);
        else
            ac_mitsubishi152.setSwingVertical(kMitsubishiHeavy88SwingVOff);
        ac_mitsubishi152.setMode(ac_mitsubishi152.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_mitsubishi152.send();
        break;

    case RAW:
        if (!last_command.power)
        {
            ac_custom.sendRaw(teachingModeIrCmds[0], teaching_mode_raw_len, RAW_FREQ);
        }
        else
        {
            ac_custom.sendRaw(teachingModeIrCmds[last_command.temperature - MAX_LOW_TEMP + 1], teaching_mode_raw_len, RAW_FREQ);
        }
        vTaskDelay(0);    // yield to scheduler, sufficient for ~100ms transmission
        break;

    case SAMSUNG_AC:
        ac_samsung.setPower(last_command.power);
        ac_samsung.setTemp(last_command.temperature);
        ac_samsung.setSwingH(last_command.swingh);
        ac_samsung.setSwing(last_command.swingv);
        ac_samsung.setFan(ac_samsung.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_samsung.setOffTimer(last_command.offtimer);
        ac_samsung.setOnTimer(last_command.ontimer);
        ac_samsung.setMode(ac_samsung.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_samsung.send();
        break;

    case VOLTAS:
        ac_voltas.setPower(last_command.power);
        ac_voltas.setTemp(last_command.temperature);
        ac_voltas.setSwingH(last_command.swingh);
        ac_voltas.setSwingV(last_command.swingv);
        ac_voltas.setFan(ac_voltas.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_voltas.setOffTime(last_command.offtimer);
        ac_voltas.setOnTime(last_command.ontimer);
        ac_voltas.setMode(ac_voltas.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_voltas.send();
        break;

    case TOSHIBA_AC:
        ac_toshiba.setPower(last_command.power);
        ac_toshiba.setTemp(last_command.temperature);
        ac_toshiba.setFan(ac_toshiba.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        if (last_command.swingv || last_command.swingh)
            ac_toshiba.setSwing(kToshibaAcSwingOn);
        else
            ac_toshiba.setSwing(kToshibaAcSwingOff);
        ac_toshiba.setMode(ac_toshiba.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_toshiba.send();
        break;

    case COOLIX:
        ac_coolix.setPower(last_command.power);
        if (!last_command.power)
        {
            // Coolix protocol: send immediately after power off, no other params
            ac_coolix.send();
            break;
        }
        ac_coolix.setTemp(last_command.temperature);
        // Mode must be set before fan for Coolix
        ac_coolix.setMode(ac_coolix.convertMode((stdAc::opmode_t)last_command.mode_num));
        // Fan must be set after mode as setMode can change fan speed
        ac_coolix.setFan(ac_coolix.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        ac_coolix.send();
        break;

    case FUJITSU_AC:
        ac_fujitsu.setPower(last_command.power);
        ac_fujitsu.setTemp(last_command.temperature);
        ac_fujitsu.setMode(ac_fujitsu.convertMode((stdAc::opmode_t)last_command.mode_num));
        ac_fujitsu.setFanSpeed(ac_fujitsu.convertFan((stdAc::fanspeed_t)last_command.fanspeed));
        if (last_command.swingh && last_command.swingv)
            ac_fujitsu.setSwing(kFujitsuAcSwingBoth);
        else if (last_command.swingh)
            ac_fujitsu.setSwing(kFujitsuAcSwingHoriz);
        else if (last_command.swingv)
            ac_fujitsu.setSwing(kFujitsuAcSwingVert);
        else
            ac_fujitsu.setSwing(kFujitsuAcSwingOff);
        ac_fujitsu.setOnTimer(last_command.ontimer * 60);
        ac_fujitsu.setOffTimer(last_command.offtimer * 60);
        ac_fujitsu.send();
        break;
    }
    // Restore original task priority
    vTaskPrioritySet(NULL, prev_priority);
    irrecv.resume();
}

/**
 * @brief Function that returns the detected protocol's name
 * @param protocol
 * @return char*
 */
const char *get_protocol_string(int16_t protocol)
{
    switch (protocol)
    {
    case CARRIER_AC64:
        return CARRIER_AC64_IR_PROTOCOL;
    case DAIKIN:
        return DAIKIN_IR_PROTOCOL;
    case DAIKIN200:
        return DAIKIN200_IR_PROTOCOL;
    case DAIKIN216:
        return DAIKIN216_IR_PROTOCOL;
    case DAIKIN2:
        return DAIKIN2_IR_PROTOCOL;
    case DAIKIN160:
        return DAIKIN160_IR_PROTOCOL;
    case DAIKIN176:
        return DAIKIN176_IR_PROTOCOL;
    case DAIKIN64:
        return DAIKIN64_IR_PROTOCOL;
    case DAIKIN152:
        return DAIKIN152_IR_PROTOCOL;
    case DAIKIN128:
        return DAIKIN128_IR_PROTOCOL;
    case HAIER_AC:
        return HAIER_AC_IR_PROTOCOL;
    case HAIER_AC176:
        return HAIER_AC176_IR_PROTOCOL;
    case HAIER_AC160:
        return HAIER_AC160_IR_PROTOCOL;
    case HITACHI_AC296:
        return HITACHI_AC296_IR_PROTOCOL;
    case HITACHI_AC:
        return HITACHI_AC_IR_PROTOCOL;
    case HITACHI_AC1:
        return HITACHI_AC1_IR_PROTOCOL;
    case HITACHI_AC424:
        return HITACHI_AC424_IR_PROTOCOL;
    case HITACHI_AC344:
        return HITACHI_AC344_IR_PROTOCOL;
    case HITACHI_AC264:
        return HITACHI_AC264_IR_PROTOCOL;
    case LG2:
        return LG2_IR_PROTOCOL;
    case LG:
        return LG_IR_PROTOCOL;
    case MITSUBISHI112:
        return MITSUBISHI112_IR_PROTOCOL;
    case MITSUBISHI136:
        return MITSUBISHI136_IR_PROTOCOL;
    case MITSUBISHI_AC:
        return MITSUBISHI_AC_IR_PROTOCOL;
    case MITSUBISHI_HEAVY_88:
        return MITSUBISHI_HEAVY_88_IR_PROTOCOL;
    case MITSUBISHI_HEAVY_152:
        return MITSUBISHI_HEAVY_152_IR_PROTOCOL;
    case SAMSUNG_AC:
        return SAMSUNG_AC_IR_PROTOCOL;
    case TOSHIBA_AC:
        return TOSHIBA_AC_IR_PROTOCOL;
    case RAW:
        return RAW_IR_PROTOCOL;
    case VOLTAS:
        return VOLTAS_IR_PROTOCOL;
    case COOLIX:
        return COOLIX_IR_PROTOCOL;
    case FUJITSU_AC:
        return FUJITSU_AC_IR_PROTOCOL;
    case UNKNOWN:
        return UNKNOWN_IR_PROTOCOL;
    case UNUSED:
        return UNUSED_IR_PROTOCOL;
    default:
        return INVALID_IR_PROTOCOL;
    }
}

/**
 * @brief Function that checks if the detected IR signal is a sendable IR Protocol
 * @param protocol
 * @return true
 * @return false
 */
bool is_sendable_protocol(decode_type_t protocol)
{
    switch (protocol)
    {
    case CARRIER_AC64:
    case DAIKIN:
    case DAIKIN200:
    case DAIKIN216:
    case DAIKIN2:
    case DAIKIN160:
    case DAIKIN176:
    case DAIKIN64:
    case DAIKIN152:
    case DAIKIN128:
    case HAIER_AC:
    case HAIER_AC176:
    case HAIER_AC160:
    case HITACHI_AC296:
    case HITACHI_AC:
    case HITACHI_AC1:
    case HITACHI_AC424:
    case HITACHI_AC344:
    case HITACHI_AC264:
    case LG2:
    case LG:
    case MITSUBISHI112:
    case MITSUBISHI136:
    case MITSUBISHI_AC:
    case MITSUBISHI_HEAVY_88:
    case MITSUBISHI_HEAVY_152:
    case SAMSUNG_AC:
    case TOSHIBA_AC:
    case VOLTAS:
    case COOLIX:
    case FUJITSU_AC:
        return true;
    default:
        return false;
    }
}

/**
 * @brief Function that checks if the detected IR signal is a decodeable IR Protocol
 * @param protocol
 * @return true
 * @return false
 */
bool is_decodeable_protocol(decode_type_t protocol)
{
    switch (protocol)
    {
    case CARRIER_AC:
    case CARRIER_AC40:
    case CARRIER_AC64:
    case CARRIER_AC84:
    case CARRIER_AC128:
    case DAIKIN:
    case DAIKIN2:
    case DAIKIN64:
    case DAIKIN128:
    case DAIKIN152:
    case DAIKIN160:
    case DAIKIN176:
    case DAIKIN200:
    case DAIKIN216:
    case DAIKIN312:
    case HAIER_AC:
    case HAIER_AC160:
    case HAIER_AC176:
    case HITACHI_AC1:
    case HITACHI_AC2:
    case HITACHI_AC3:
    case HITACHI_AC:
    case HITACHI_AC264:
    case HITACHI_AC296:
    case HITACHI_AC344:
    case HITACHI_AC424:
    case LG:
    case LG2:
    case MITSUBISHI:
    case MITSUBISHI_AC:
    case MITSUBISHI2:
    case MITSUBISHI112:
    case MITSUBISHI136:
    case SAMSUNG_AC:
    case SAMSUNG:
    case SAMSUNG36:
    case TOSHIBA_AC:
    case VOLTAS:
    case COOLIX:
    case FUJITSU_AC:
        return true;
    default:
        return false;
    }
}

/**
 * @brief Function that fetches the info of Power, Temperature, Fanspeed and Mode
 * from received IR Signal.
 * @warning Both teaching mode and Lockign feature depend on the Temperature value that was detected,
 * if that fails, then it's FATAL
 * @retval true = If Temperature parse was success
 * @retval false = If Temperature parse was failure
 */
bool isFetchControlInfoSuccessful(const char *description)
{
    ESP_LOGW(IR_TAG, "isFetchControlInfoSuccessful: description='%s'", description);
    /*Initializing to Defaults*/
    strcpy(ac_manual_control_t.temperature, "");
    strcpy(ac_manual_control_t.power, "");
    strcpy(ac_manual_control_t.fan, "");
    strcpy(ac_manual_control_t.mode, "");

    ac_manual_control_t.power_value = -1;
    ac_manual_control_t.fanspeed_value = -1;
    ac_manual_control_t.temperature_value = -1;

    ac_manual_control_t.power_err = SUCCESS;
    ac_manual_control_t.fanspeed_err = SUCCESS;
    ac_manual_control_t.mode_err = SUCCESS;
    ac_manual_control_t.temperature_err = SUCCESS;

    /*Starting Decoding Power, Mode, Fanspeed, Temperature Info*/
    if (strstr(description, "Power"))
    {
        snprintf(ac_manual_control_t.power, sizeof(ac_manual_control_t.power), (strstr(description, "Power") + 7));
        if (strstr(ac_manual_control_t.power, "On"))
            ac_manual_control_t.power_value = 1;
        else
            ac_manual_control_t.power_value = 0;
    }
    else
    {
        ac_manual_control_t.power_err = POWER_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
        ac_manual_control_t.power_value = -1;
    }

    if (strstr(description, "Mode:"))
    {
        if (strstr(description, "(Cool)"))
            strcpy(ac_manual_control_t.mode, COOL_MODE_STR);
        else if (strstr(description, "(Heat)"))
            strcpy(ac_manual_control_t.mode, HEAT_MODE_STR);
        else if (strstr(description, "(Auto)"))
            strcpy(ac_manual_control_t.mode, AUTO_MODE_STR);
        else if (strstr(description, "(Fan)"))
            strcpy(ac_manual_control_t.mode, FAN_MODE_STR);
        else if (strstr(description, "(Dry)"))
            strcpy(ac_manual_control_t.mode, DRY_MODE_STR);
        else
        {
            ac_manual_control_t.mode_err = MODE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
            strcpy(ac_manual_control_t.mode, "-1");
        }
    }
    else
    {
        ac_manual_control_t.mode_err = MODE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
        strcpy(ac_manual_control_t.mode, "-1");
    }

    if (strstr(description, "Fan"))
    {
        snprintf(ac_manual_control_t.fan, sizeof(ac_manual_control_t.fan), (strstr(description, "Fan") + 5));
        ac_manual_control_t.fanspeed_value = atoi(ac_manual_control_t.fan);
    }
    else
    {
        ac_manual_control_t.fanspeed_err = FANSPEED_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
        ac_manual_control_t.fanspeed_value = -1;
    }

    if (strstr(description, "Temp"))
    {
        snprintf(ac_manual_control_t.temperature, sizeof(ac_manual_control_t.temperature), (strstr(description, "Temp") + 6));
        ac_manual_control_t.temperature_value = atoi(ac_manual_control_t.temperature);
        // Populate currACState fields from last_command
        ac_manual_control_t.swingh = last_command.swingh;
        ac_manual_control_t.swingv = last_command.swingv;
        ac_manual_control_t.ontimer = last_command.ontimer;
        ac_manual_control_t.offtimer = last_command.offtimer;
        ac_manual_control_t.locking = last_command.locking;
        ac_manual_control_t.upperTemperatureLimit = last_command.upperTemperatureLimit;
        ac_manual_control_t.lowerTemperatureLimit = last_command.lowerTemperatureLimit;
        ac_manual_control_t.ambientTemperatureAnalog = read_analog_temperature_sensor();
        ac_manual_control_t.ambientTemperatureDigital = read_digital_temperature_sensor();
        return true;
    }
    else
    {
        ac_manual_control_t.temperature_err = TEMPERATURE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
        ac_manual_control_t.temperature_value = -1;
        return false;
    }
}

/**
 * @brief Function that performs the AC locking feature
 *
 * Changes made:
 * Issue 1 - Power OFF: Locking only triggers when Power=ON.
 *            If user presses Power OFF, always accept — never override.
 * Issue 2 - Fan Mode: Locking never triggers when mode is Fan.
 *            Checked both detected mode from remote and last cloud-set mode.
 * Issue 3 - Temperature not in IR description: If isFetchControlInfoSuccessful()
 *            returns false (e.g. Fujitsu Power OFF command has no temp in description),
 *            skip locking entirely and still send ACK to cloud.
 * Issue 4 - mode_num not updated: When user changes mode via physical remote,
 *            last_command.mode_num is now updated alongside mode_str so NVS
 *            stays consistent after reboot.
 * Issue 6 - Power OFF during override: When locking correction fires ir_transmit(),
 *            last_command.power is forced to 1 (ON) so the AC actually turns on
 *            at the locked temperature instead of receiving a Power OFF command.
 */
void locking_feature(const char *description)
{
    // Debug log — print all values before locking decision
    ESP_LOGW(IR_TAG, "locking_feature called: locking=%d power_val=%d mode=%s temp_val=%d limits=[%d-%d]",
        last_command.locking,
        ac_manual_control_t.power_value,
        ac_manual_control_t.mode,
        ac_manual_control_t.temperature_value,
        last_command.lowerTemperatureLimit,
        last_command.upperTemperatureLimit);

    // Issue 3 — If temperature could not be parsed from IR signal,
    // skip locking check entirely and fall through to ACK
    if (!isFetchControlInfoSuccessful(description))
    {
        ESP_LOGW(IR_TAG, "Locking feature: temperature not in IR description — skipping locking check");
    }
    else
    {
        if (last_command.locking &&                                         // Locking must be enabled by cloud
            ac_manual_control_t.power_value == 1 &&                        // Issue 1: Only lock when Power=ON
            strcasecmp(ac_manual_control_t.mode, FAN_MODE_STR) != 0 &&     // Issue 2: Skip lock if remote pressed Fan mode
            strcasecmp(last_command.mode_str, FAN_MODE_STR) != 0 &&        // Issue 2: Skip lock if last cloud command was Fan mode
            !(ac_manual_control_t.temperature_value >= last_command.lowerTemperatureLimit &&
              ac_manual_control_t.temperature_value <= last_command.upperTemperatureLimit)) // Temperature outside locked range
        {
            ESP_LOGW(IR_TAG, "AC temperature limit exceeded  (Current set Temperature : %d) | (Limits %d - %d)",
                     ac_manual_control_t.temperature_value,
                     last_command.lowerTemperatureLimit,
                     last_command.upperTemperatureLimit);

            // Clamp to nearest limit boundary
            // If below lower limit → set to lower limit
            // If above upper limit → set to upper limit
            if (ac_manual_control_t.temperature_value < last_command.lowerTemperatureLimit)
            {
                ESP_LOGW(IR_TAG, "Temp %d below lower limit %d — clamping to lower limit",
                    ac_manual_control_t.temperature_value, last_command.lowerTemperatureLimit);
                last_command.temperature = last_command.lowerTemperatureLimit;
            }
            else if (ac_manual_control_t.temperature_value > last_command.upperTemperatureLimit)
            {
                ESP_LOGW(IR_TAG, "Temp %d above upper limit %d — clamping to upper limit",
                    ac_manual_control_t.temperature_value, last_command.upperTemperatureLimit);
                last_command.temperature = last_command.upperTemperatureLimit;
            }

            ESP_LOGW(IR_TAG, "Setting AC Temperature to : %d", last_command.temperature);
            last_command.power = 1;    // Issue 6: Force Power ON so AC turns on at locked temp
            ir_transmit();
        }
        else
        {
            // Locking not triggered — accept what user pressed and update last_command
            last_command.power = ac_manual_control_t.power_value;
            last_command.temperature = ac_manual_control_t.temperature_value;
            last_command.fanspeed = ac_manual_control_t.fanspeed_value;
            strcpy(last_command.mode_str, ac_manual_control_t.mode);
            // Issue 4: Update mode_num alongside mode_str for NVS consistency after reboot
            if      (strcasecmp(ac_manual_control_t.mode, "Cool") == 0) last_command.mode_num = 1;
            else if (strcasecmp(ac_manual_control_t.mode, "Hot")  == 0) last_command.mode_num = 2;
            else if (strcasecmp(ac_manual_control_t.mode, "Auto") == 0) last_command.mode_num = 0;
            else if (strcasecmp(ac_manual_control_t.mode, "Dry")  == 0) last_command.mode_num = 3;
            else if (strcasecmp(ac_manual_control_t.mode, "Fan")  == 0) last_command.mode_num = 4;
            set_blob_in_nvs_flash(IR_HANDLE, NVS_LAST_COMMAND_KEY, &last_command, sizeof(CommandStruct));
        }
    }

#if (IS_GWY)
    // Gateway: publish manual AC control ACK directly to MQTT
    generate_ack(GWY_MANUAL_AC_CONTROL_ACK, NULL);
#endif

#if (!IS_GWY)
    // Node: send manual AC control ACK to provisioner over BLE Mesh
    send_manual_control_ack_to_provisioner();
#endif
}

/**
 * @brief Function that initializes the value for teaching mode process
 * @param startingTemp Starting Temperature of Teaching Mode
 * @param endingTemp Ending Temperature of Teaching Mode
 */
void teaching_mode_init(uint8_t startingTemp, uint8_t endingTemp)
{
    esp_err_t err = nvs_flash_erase_partition(IR_NVS_PARTITION_NAME);
    if(err) ESP_LOGE(IR_TAG, "Failed to clear %s partition - %s", IR_NVS_PARTITION_NAME, esp_err_to_name(err));

    err = nvs_flash_init_partition(IR_NVS_PARTITION_NAME);
    if(err) ESP_LOGE(IR_TAG, "Failed to initialize %s partition - %s", IR_NVS_PARTITION_NAME, esp_err_to_name(err));

    err = nvs_open_from_partition(IR_NVS_PARTITION_NAME, IR_NVS_NAMESPACE, NVS_READWRITE, &ir_nvs_handle);
    if(err) ESP_LOGE(IR_TAG, "Failed to open handle to %s partition - %s", IR_NVS_PARTITION_NAME, esp_err_to_name(err));

    if(err) {
        teaching_mode_t.errorCode = IR_PARTITION_INIT_FAILED;
        goto here;
    }

    teaching_in_progress = true; update_led_status();
    ESP_LOGW(IR_TAG, "Device Entered Teaching mode");
    
    teaching_mode_t.errorCode = ENTERED_TEACHING_MODE;
    teaching_mode_t.expectedTemperature = startingTemp;
    teaching_mode_t.commandIndex = startingTemp - MAX_LOW_TEMP;
    teaching_mode_t.remainingCommands = endingTemp - startingTemp + 2;
    strcpy(teaching_mode_t.lastCommand, "");
    strcpy(teaching_mode_t.nextCommand, TEACHING_MODE_SEQUENCE[0]);
    set_number_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_STARTING_TEMPERATURE_KEY, startingTemp, UINT8_SIZE);
    set_number_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_ENDING_TEMPERATURE_KEY, endingTemp, UINT8_SIZE);

    if(!teaching_mode_t.errorCheckEnabled) 
        memset(teaching_mode_cmd_recvd_array, false, sizeof(teaching_mode_cmd_recvd_array));

here:
#if (IS_GWY)
    generate_ack(GWY_TEACHING_MODE, NULL);
#else
    send_teaching_mode_ack_to_provisioner();
#endif
}

/**
 * @brief Function that exits the device from teaching mode
 * @param success - Flag indicating if its a successful exit or forced exit
 * if successful exit, then teaching mode is completed and hence we can set
 * the configured flag, if not, then don't
 */
void exit_teaching_mode(bool success)
{
    teaching_mode_t.errorCode = EXITED_TEACHING_MODE;
    strcpy(teaching_mode_t.nextCommand, "");
    teaching_in_progress = false;

    if (success)
    {
        pull_ir_cmd_data(); // Pull the data from nvs flash to RAM for immediate usage
        configured = true;
        set_number_in_nvs_flash(GENERAL_HANDLE, NVS_CONFIGURED_KEY, 1, UINT8_SIZE);
        ir_protocol_num = RAW;
        strcpy(ir_protocol, get_protocol_string(ir_protocol_num));
        set_number_in_nvs_flash(IR_HANDLE, NVS_IR_PROTOCOL_KEY, ir_protocol_num, INT16_SIZE);
        ESP_LOGI(IR_TAG, "Teaching mode completed successfully");
    }
    else ESP_LOGW(IR_TAG, "Quitting teaching mode without completion");
    
     update_led_status();
    
#if (IS_GWY)
    generate_ack(GWY_TEACHING_MODE, NULL);
#endif
#if (!IS_GWY)
    send_teaching_mode_ack_to_provisioner();
#endif
}

/**
 * @brief Get the remaining teaching mode cmds count
 * @return uint8_t 
 */
uint8_t get_remaining_teaching_mode_cmds_count()
{
    uint8_t count = 0;
    for(uint8_t i=0;i<MAX_CMDS_IN_TEACHING_MODE;i++)
    {
        if(teaching_mode_cmd_recvd_array[i]) count++;
    }
    uint8_t total_commands_req = teaching_mode_t.endingTemperature-teaching_mode_t.startingTemperature + 2;
    uint8_t remainingCommands = total_commands_req - count;
    return remainingCommands;
}

/**
 * @brief Function that performs the teaching process with error check support
 * - 1) Detects IR Signal
 * - 2) Decodes information from IR Signal = Temperature, Power
 * - 3) Checks if the expected IR Signal
 * - 4) If Yes, saves it to nvs flash
 * - 5) Steps 1 to 4 is repeated until expected number of IR Signals have been received
 * - 6) Quits device from Teaching mode
 */
void perform_teaching_process_with_error_checking(const char *description)
{
    /*Let's pause the IR Reception until we process the recevied ir signal*/
    irrecv.pause();

    if (isFetchControlInfoSuccessful(description) || teaching_mode_t.commandsReceived == 0)
    {
        // We need to neglect the first value in the received signal
        memmove((void *)results.rawbuf, (const void *)&results.rawbuf[1], (results.rawlen - 1) * sizeof(results.rawbuf[0]));
        results.rawlen -= 1;

        /*I'm not sure why we have to do this, but we've to multiply the received signal values with 2*/
        /*For some reason, if we transmit a signal of length 3.5ms, the receiver side prints it as 1.75ms*/
        for (int i = 0; i < results.rawlen; i++)
        {
            results.rawbuf[i] = results.rawbuf[i] *2;
        }

        /*If this is the first IR Signal*/
        if (teaching_mode_t.commandsReceived == 0)
        {
            teaching_mode_raw_len = results.rawlen;
            set_number_in_nvs_flash(IR_HANDLE, NVS_RAWLEN_KEY, teaching_mode_raw_len, UINT16_SIZE);

            /*Let's store raw values to nvs flash*/
            set_blob_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_CMD_KEYS[0], (const void *)results.rawbuf, teaching_mode_raw_len * sizeof(uint16_t));

            /**
             * @warning We're only comparing power here and not temperature, because, for some AC remotes
             * like LG2, when sending power OFF command, the decoded string doesn't seem to contain temperature at all.
             *
             */
            if (ac_manual_control_t.power_value == 0)
            {
                ESP_LOGI(IR_TAG, "Detected Rawlen : %d | Detected Temperature : %d | Detected Power : %d | Required Temperature : %d | Required Power : %d",
                         results.rawlen, ac_manual_control_t.temperature_value, ac_manual_control_t.power_value, teaching_mode_t.expectedTemperature, 0);
                /*We've received a valid first IR Signal*/
                teaching_mode_t.commandsReceived++;
                teaching_mode_t.commandIndex++;
                teaching_mode_t.remainingCommands--;
                teaching_mode_t.errorCode = SUCCESS;
                strcpy(teaching_mode_t.lastCommand, TEACHING_MODE_SEQUENCE[0]);
                strcpy(teaching_mode_t.nextCommand, TEACHING_MODE_SEQUENCE[teaching_mode_t.commandIndex]);
            }
            else
            {
                ESP_LOGW(IR_TAG, "Detected Rawlen : %d | Detected Temperature : %d | Detected Power : %d | Required Temperature : %d | Required Power : %d",
                         results.rawlen, ac_manual_control_t.temperature_value, ac_manual_control_t.power_value, teaching_mode_t.expectedTemperature, 0);
                led_set_state(LED_STATE_INVALID_OPERATION);
                teaching_mode_t.errorCode = FAILURE;
            }
        }
        else
        {
            if (ac_manual_control_t.power_value == 1 && ac_manual_control_t.temperature_value == teaching_mode_t.expectedTemperature)
            {
                ESP_LOGI(IR_TAG, "Detected Rawlen : %d | Detected Temperature : %d | Detected Power : %d | Required Temperature : %d | Required Power : %d",
                         results.rawlen, ac_manual_control_t.temperature_value, ac_manual_control_t.power_value, teaching_mode_t.expectedTemperature, 1);

                /*Let's store raw values to nvs flash*/
                set_blob_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_CMD_KEYS[teaching_mode_t.commandIndex], (const void *)results.rawbuf, teaching_mode_raw_len * sizeof(uint16_t));

                /*We've received a valid IR Signal*/
                teaching_mode_t.commandsReceived++;
                teaching_mode_t.commandIndex++;
                teaching_mode_t.remainingCommands--;
                teaching_mode_t.errorCode = SUCCESS;
                teaching_mode_t.expectedTemperature++;
                strcpy(teaching_mode_t.lastCommand, teaching_mode_t.nextCommand);
                strcpy(teaching_mode_t.nextCommand, TEACHING_MODE_SEQUENCE[teaching_mode_t.commandIndex]);
            }
            else
            {
                ESP_LOGE(IR_TAG, "Detected Rawlen : %d | Detected Temperature : %d | Detected Power : %d | Required Temperature : %d | Required Power : %d",
                         results.rawlen, ac_manual_control_t.temperature_value, ac_manual_control_t.power_value, teaching_mode_t.expectedTemperature, 1);
                led_set_state(LED_STATE_INVALID_OPERATION);
                teaching_mode_t.errorCode = FAILURE;
            }
        }
    }
    else
    {
        led_set_state(LED_STATE_INVALID_OPERATION);
        teaching_mode_t.errorCode = TEMPERATURE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING;
    }
    if (teaching_mode_t.remainingCommands != 0)
#if (IS_GWY)
        generate_ack(GWY_TEACHING_MODE, NULL);
#else
        send_teaching_mode_ack_to_provisioner();
#endif

    /*Teaching mode has been successfully completed*/
    else
        exit_teaching_mode(true);

    irrecv.resume();
}

/**
 * @brief Function that performs the teaching process without error check support
 * - 1) Detects IR Signal
 * - 2) Saves it to nvs flash
 * - 3) Steps 1 & 2 is repeated until expected number of IR Signals have been received
 * - 6) Quits device from Teaching mode
 */
void perform_teaching_process_without_error_checking()
{
    /*Let's pause the IR Reception until we process the recevied ir signal*/
    irrecv.pause();

    // We need to neglect the first value in the received signal
    memmove((void *)results.rawbuf, (const void *)&results.rawbuf[1], (results.rawlen - 1) * sizeof(results.rawbuf[0]));
    results.rawlen -= 1;

    /*I'm not sure why we have to do this, but we've to multiply the received signal values with 2*/
    /*For some reason, if we transmit a signal of length 3.5ms, the receiver side prints it as 1.75ms*/
    for (int i = 0; i < results.rawlen; i++)
    {
        results.rawbuf[i] = results.rawbuf[i]*2;
    }

    teaching_mode_raw_len = results.rawlen;
    set_number_in_nvs_flash(IR_HANDLE, NVS_RAWLEN_KEY, teaching_mode_raw_len, UINT16_SIZE);

    if(!teaching_mode_t.power)
    {
        /*Let's store raw values to nvs flash*/
        set_blob_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_CMD_KEYS[0], (const void *)results.rawbuf, teaching_mode_raw_len * sizeof(uint16_t));
        strcpy(teaching_mode_t.lastCommand, TEACHING_MODE_SEQUENCE[0]);
        teaching_mode_cmd_recvd_array[0] = true;
    }
    else
    {
        teaching_mode_t.commandIndex = teaching_mode_t.temperature - MAX_LOW_TEMP + 1;
        set_blob_in_nvs_flash(IR_HANDLE, NVS_TEACHING_MODE_CMD_KEYS[teaching_mode_t.commandIndex], (const void *)results.rawbuf, teaching_mode_raw_len * sizeof(uint16_t));
        strcpy(teaching_mode_t.lastCommand, TEACHING_MODE_SEQUENCE[teaching_mode_t.commandIndex]);
        teaching_mode_cmd_recvd_array[teaching_mode_t.commandIndex] = true;
    }

    teaching_mode_t.remainingCommands = get_remaining_teaching_mode_cmds_count();
    teaching_mode_t.errorCode = SUCCESS;
    
    // ADD THIS HERE — after remainingCommands is updated:
    ESP_LOGW(IR_TAG, "Stored slot [%d] | remainingCommands=%d | recvd_array=[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
        teaching_mode_t.power ? teaching_mode_t.commandIndex : 0,
        teaching_mode_t.remainingCommands,
        teaching_mode_cmd_recvd_array[0],  teaching_mode_cmd_recvd_array[1],
        teaching_mode_cmd_recvd_array[2],  teaching_mode_cmd_recvd_array[3],
        teaching_mode_cmd_recvd_array[4],  teaching_mode_cmd_recvd_array[5],
        teaching_mode_cmd_recvd_array[6],  teaching_mode_cmd_recvd_array[7],
        teaching_mode_cmd_recvd_array[8],  teaching_mode_cmd_recvd_array[9],
        teaching_mode_cmd_recvd_array[10], teaching_mode_cmd_recvd_array[11],
        teaching_mode_cmd_recvd_array[12], teaching_mode_cmd_recvd_array[13]);
        
    if (teaching_mode_t.remainingCommands != 0)
#if (IS_GWY)
        generate_ack(GWY_TEACHING_MODE, NULL);
#else
        send_teaching_mode_ack_to_provisioner();
#endif

    /*Teaching mode has been successfully completed*/
    else
        exit_teaching_mode(true);

    irrecv.resume();
}

/**
 * @brief Thread that takes care of decoding IR signals
 * @param args
 */
void ir_recv_task(void *args)
{
    ESP_LOGI(IR_TAG, "Starting IR Recv Task");
    irrecv.setUnknownThreshold(12);
    irrecv.setTolerance(kTolerancePercentage);
    irrecv.enableIRIn();
    while (1)
    {
        if (irrecv.decode(&results))
        {
            decode_type_t protocol = UNKNOWN;
            if (results.overflow)
                ESP_LOGW(IR_TAG, "IR Buffer overflow");
            ESP_LOGW(IR_TAG, "kTolerancePercentage : %d", kTolerancePercentage);
            ESP_LOGW(IR_TAG, "%s", resultToHumanReadableBasic(&results, &protocol).c_str());
            ESP_LOGW(IR_TAG, "Protocol Number : %d | Rawlen : %d", protocol, results.rawlen);
            String description = IRAcUtils::resultAcToString(&results);
            if (description.length())
                ESP_LOGW(IR_TAG, "%s\n", description.c_str());

            if ((results.rawlen > 20 && !configured) || teaching_in_progress)
            {
                ESP_LOGI(IR_TAG, "Detected IR Signal values : ");
                for (int i = 0; i < results.rawlen; i++)
                {
                    printf("%d ", results.rawbuf[i]);
                }
                printf("\n");
            }

            /**  @warning DO NOT CHANGE THE FOLLOWING ORDER
             * 1) locking Feature
             * 2) Teaching Mode
             * 3) AC Remote Configuration
             * 4) Unsupported AC Remote Check
             */

            /*Locking Feature*/
            if (configured && !teaching_in_progress &&
                ((protocol == ir_protocol_num) || (ir_protocol_num == RAW && results.rawlen == teaching_mode_raw_len)) &&
                description.length())
            {
                led_set_state(LED_STATE_IR_SIGNAL_DETECTED);
                locking_feature(description.c_str());
                continue;
            }

            /*Teaching Mode*/
            else if (teaching_in_progress)
            {
                if (teaching_mode_t.errorCheckEnabled)
                    perform_teaching_process_with_error_checking(description.c_str());
                else
                    perform_teaching_process_without_error_checking();
                continue;
            }

            CommandStruct ack;
            /*AC Remote Configuration Process*/
            if (!configured && !teaching_in_progress)
            {
                
                ack.irProtocolNum = protocol;
#if (IS_GWY)
                ack.packetid = GWY_CONF_ACK;
#else
                ack.packetid = NODE_CONF_ACK;
                strcpy(ack.deviceName, serialNoStr);
                ack.elemaddr = last_command.elemaddr;
#endif
                if (is_sendable_protocol(protocol))
                {
                    configured = true; 
                    update_led_status();
                    ir_protocol_num = protocol;
                    strcpy(ir_protocol, get_protocol_string(ir_protocol_num));
                    set_number_in_nvs_flash(IR_HANDLE, NVS_IR_PROTOCOL_KEY, ir_protocol_num, INT16_SIZE);
                    set_number_in_nvs_flash(GENERAL_HANDLE, NVS_CONFIGURED_KEY, 1, UINT8_SIZE);

                    ack.errorcode = SUCCESS;
#if (IS_GWY)
                    generate_ack(GWY_CONF_ACK, &ack);
#else
                    send_ack_to_provisioner(NODE_CONF_ACK, &ack);
#endif
                    ESP_LOGI(IR_TAG, "AC Remote configuration successful");
                    goto here;
                }

                led_set_state(LED_STATE_INVALID_OPERATION);

                if (is_decodeable_protocol(protocol))
                {
                    ESP_LOGE(IR_TAG, "AC Remote Configuration Failed | Protocol is deocdeable-only");
                    ESP_LOGW(IR_TAG, "Move on to Teaching mode with error checking");
                    teaching_mode_t.errorCheckEnabled = 1;
                    ack.errorcode = IR_PROTOCOL_DECODEABLE_ONLY;
#if (IS_GWY)
                    generate_ack(GWY_CONF_ACK, &ack);
#else
                    send_ack_to_provisioner(NODE_CONF_ACK, &ack);
#endif
                }
                else
                {
                    ESP_LOGE(IR_TAG, "AC Remote Configuration Failed | Protocol is neither decodeable nor sendable");
                    ESP_LOGW(IR_TAG, "Move on to Teaching mode without error checking");
                    ack.errorcode = IR_PROTOCOL_FULLY_UNSUPPORTED;
                    teaching_mode_t.errorCheckEnabled = 0;
#if (IS_GWY)
                    generate_ack(GWY_CONF_ACK, &ack);
#else
                    send_ack_to_provisioner(NODE_CONF_ACK, &ack);
#endif
                }
            }
        here:
            yield();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Function that takes are of setting the configured to false,
 * and updating the flash. Common function to both provisioner and node
 *
 */
void handle_reconfiguration()
{
    ir_protocol_num = -1;
    strcpy(ir_protocol, get_protocol_string(ir_protocol_num));
    set_number_in_nvs_flash(GENERAL_HANDLE, NVS_CONFIGURED_KEY, 0, UINT8_SIZE);
    set_number_in_nvs_flash(IR_HANDLE, NVS_IR_PROTOCOL_KEY, ir_protocol_num, INT16_SIZE);
    configured = 0;
    update_led_status();
}

/**
 * @brief Function that takes care of configuring teaching mode.
 * Common function to both provisioner and node
 * @param cmd_struct
 */
void handle_configuring_teaching_mode(action_type_t type, CommandStruct *cmd_struct)
{
    if (type == DUE_TO_BUTTON_PRESS)
    {
#if (IS_GWY)
        teaching_mode_t.packetid = GWY_TEACHING_MODE;
#endif
        teaching_mode_t.errorCode = SUCCESS;
        teaching_mode_t.msgseqno = BUTTON_PRESS_MSGSEQNO;
        teaching_mode_t.startingTemperature = MAX_LOW_TEMP;
        teaching_mode_t.endingTemperature = MAX_HIGH_TEMP;

        if (teaching_in_progress)
        {
            teaching_mode_t.teachingStart = 0;
            exit_teaching_mode(false);
        }
        else
        {
            teaching_mode_t.teachingStart = 1;
            teaching_mode_init(teaching_mode_t.startingTemperature, teaching_mode_t.endingTemperature);
        }
    }

    /*MQTT Packet case*/
    else
    {
        teaching_mode_t.packetid = cmd_struct->packetid;
        teaching_mode_t.errorCode = cmd_struct->errorcode;
        teaching_mode_t.msgseqno = cmd_struct->msgseqno;
        teaching_mode_t.teachingStart = cmd_struct->teachingStart;
        teaching_mode_t.startingTemperature = cmd_struct->startingTemperature;
        teaching_mode_t.endingTemperature = cmd_struct->endingTemperature;
        if (!teaching_mode_t.teachingStart)
        {
            exit_teaching_mode(false);
        }
        else
        {
            teaching_mode_init(teaching_mode_t.startingTemperature, teaching_mode_t.endingTemperature);
        }
    }
}