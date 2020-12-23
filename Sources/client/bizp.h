#ifndef __BIZP_H
#define __BIZP_H

struct bizp_handler
{
	void (*on_get_property)(char *json);
	void (*on_set_property)(char *json);
	void (*on_run_command)(char *json);
};

void bizp_init(void);
void bizp_bind(struct bizp_handler *handler);
int  bizp_connect(void);
int  bizp_disconnect(void);
int  bizp_sync_time(void);
int  bizp_send_heart(void);
int  bizp_send_event(char *event, char *json, int size);
int  bizp_reply_get_property(char *json, int size);
int  bizp_reply_set_property(char *json, int size);
int  bizp_reply_run_command(char *json, int size);

#endif
