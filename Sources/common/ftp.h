#ifndef __FTP_H
#define __FTP_H

void ftp_init(void);
int  ftp_login(char *addr, int port, char *username, char *password);
int  ftp_filesize(char *name);
int  ftp_download(char *name, void *buf, int len);
int  ftp_upload(char *name, void *buf, int len);
void ftp_quit(void);

#endif
