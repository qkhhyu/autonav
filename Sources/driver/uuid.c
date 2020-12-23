#include <string.h>
#include "stm32f4xx.h"
#include "uuid.h"

#define UUID_BASE	((volatile uint32_t*)0x1FFF7A10)

void uuid_read(uint8_t uuid[12])
{
	uint32_t id;
	id = *UUID_BASE;
	memcpy(uuid,&id,4);
	id = *(UUID_BASE+1);
	memcpy(uuid+4,&id,4);
	id = *(UUID_BASE+2);
	memcpy(uuid+8,&id,4);
}
