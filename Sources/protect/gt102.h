#ifndef __GT102_H
#define __GT102_H

void gt102_init(void);
int  gt102_version(uint32_t* output);
int  gt102_set_uid(uint8_t usid[8]);
int  gt102_set_key(uint8_t key[8]);
int  gt102_set_page(uint8_t page, uint8_t data[32]);
int  gt102_auth_device(uint8_t page, uint8_t input[8], uint8_t output[32]);

#endif
