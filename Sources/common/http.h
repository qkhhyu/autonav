#ifndef __HTTP_H
#define __HTTP_H

typedef void * http_t;

http_t http_create(void);
void   http_delete(http_t http);

void   http_request_init(http_t http, char *mothod, char *uri);
int    http_request_send(http_t http, char *host, int port, int timeout);
void   http_request_set_head(http_t http, char *key, char *value);
void   http_request_set_body(http_t http, char *body, int size);

int    http_response_get_code(http_t http);
int    http_response_get_head(http_t http, char *key, char *value, int size);
int    http_response_get_body(http_t http, char **body, int *size);

#endif
