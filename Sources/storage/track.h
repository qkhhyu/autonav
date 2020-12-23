#ifndef __TRACK_H
#define __TRACK_H

//轨迹坐标
struct track
{
	double  longitude;    //经度
	double  latitude;     //纬度
};

void track_init(void);
int  track_write(int flag, struct track *track);
int  track_read(struct track *track, int count);

#endif
