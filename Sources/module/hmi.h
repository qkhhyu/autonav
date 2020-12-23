#ifndef __HMI_H
#define __HMI_H

struct hmi_handler
{
	void (*on_hid_connect)(uint8_t state);
	int  (*on_hid_control_sail)(int16_t speed1, int16_t speed2, int16_t dir);
	int  (*on_hid_control_throw)(int16_t speed1, int16_t speed2);
	int  (*on_cfg_write)(uint8_t type, uint8_t *data, int size);
	int  (*on_cfg_read)(uint8_t type, uint8_t *data);
	int  (*on_cfg_control)(uint8_t type, uint8_t *data, int size);
};

void hmi_init(void);
void hmi_bind(struct hmi_handler *handler);
void hmi_dbg_dump(char *text, void *mem, int size);
void hmi_dbg_puts(char *fmt, ...);
void hmi_hid_send_gps(struct gps *gps);
void hmi_hid_send_dist(struct dist *dist);
void hmi_hid_send_batt(struct batt batt[3]);
void hmi_hid_send_sail(struct motor sail[3]);
void hmi_hid_send_feed(struct motor feed[2]);

#define LOG_HMI hmi_dbg_puts

#endif
