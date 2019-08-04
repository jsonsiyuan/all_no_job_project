

#ifndef DEVICE_MANAGER_H_
#define DEVICE_MANAGER_H_



typedef struct{
	unsigned char  id[1];
}NDevice;
int dev_man_init(void);
int dev_man_add(int type, NDevice newdev);
int dev_man_modify(int type,NDevice newdev);
int dev_man_get_device_by_id(int type,unsigned char *id,NDevice *outDev);
int dev_man_del_by_id(int type,unsigned char *id);
int dev_man_get_devices_num(int type);





#endif /* DEVICE_MANAGER_H_ */
