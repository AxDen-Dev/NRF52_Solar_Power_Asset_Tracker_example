#ifndef BG96_DEFINE_H_
#define BG96_DEFINE_H_

#define BG96_AT_ECHO "AT"

#define BG96_AT_ECHO_OFF "ATE0"

#define BG96_AT_POWER_DOWN "AT+QPOWD=1"

#define BG96_AT_SIM_ICCID_READ "AT+QCCID"

#define BG96_AT_SIM_READY_READ "AT+CPIN?"

#define BG96_AT_CSQ_READ "AT+CSQ"

#define BG96_AT_NETWORK_REGISTRAION_ENABLE "AT+CEREG=2"

#define BG96_AT_POWER_SAVE_MODE_DISABLE "AT+CPSMS=0"

#define BG96_AT_PDP_ACTIVATE "AT+QIACT=1,1"

#define BG96_AT_PDP_STATE_READ "AT+QIACT?"

#define BG96_AT_NETWORK_STATE_READ "AT+CREG?"

#define BG96_AT_2G_3G_NETWORK_STATE_READ "AT+CGREG?"

#define BG96_AT_4G_NETWORK_STATE_READ "AT+CEREG?"

#define BG96_AT_APN_IP_PROTOCOL_READ "AT+QICSGP=1"

#endif /* BG96_DEFINE_H_ */
