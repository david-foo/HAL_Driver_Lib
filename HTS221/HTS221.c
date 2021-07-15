#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <dirent.h>

#define I2C_FILE_NAME "/dev/i2c-3"

#define HTS221_ADDR  0x5F
#define HTS211_AV_CONF      0x10
#define HTS221_CTRL_REG1	0x20
#define HTS221_CTRL_REG2	0x21
#define HTS221_CTRL_REG3	0x22
#define HTS221_STATUS_REG	0x27
#define HTS221_HUMIDITY_OUT_L	0x28
#define HTS221_HUMIDITY_OUT_H	0x29
#define HTS221_TEMP_OUT_L	0x2A
#define HTS221_TEMP_OUT_H	0x2B

void HTS221_Init(void);
unsigned char	HTS221_Get_Temperature(int16_t* temperature);
unsigned char	HTS221_Get_Humidity(int16_t* humidity);

static int device_exist = 0;
static int i2c_file;

int init_i2c(void)
{
	if ((i2c_file = open(I2C_FILE_NAME, O_RDWR)) < 0) {
		return 1;
	}
	return 0;
}

void exit_i2c(void)
{
	close(i2c_file);
}

static int set_i2c_register(int file,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char value) {

    unsigned char outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    /* The first byte indicates which register we'll write */
    outbuf[0] = reg;

    /*
     * The second byte indicates the value to write.  Note that for many
     * devices, we can write multiple, sequential registers at once by
     * simply making outbuf bigger.
     */
    outbuf[1] = value;

    /* Transfer the i2c packets to the kernel and verify it worked */
    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send I2C data");
        return 1;
    }

    return 0;
}


static int get_i2c_register(int file,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send I2C data");
        return 1;
    }
    *val = inbuf;

    return 0;
}


void hts221_set_reg(unsigned char reg, unsigned char val)
{
	set_i2c_register(i2c_file,HTS221_ADDR,reg,val);
}

void hts11_get_reg(unsigned char reg, unsigned char * val)
{
	get_i2c_register(i2c_file,HTS221_ADDR,reg,val);
}

/* ���ù���ģʽ����ʼ��HTS221 */
void HTS221_Init()
{
	unsigned char cmd = 0;

	//���÷ֱ���
	cmd = 0x3F;
	hts221_set_reg(HTS211_AV_CONF,cmd);

	//���õ�Դ�����ݿ����ģʽ�������������
	cmd = 0x84;
	hts221_set_reg(HTS221_CTRL_REG1,cmd);

	//�������ݴ洢�鸴λģʽ���ر��ڲ�����
	cmd = 0x00;
	hts221_set_reg(HTS221_CTRL_REG2,cmd);

	//�ر������������ź�
	cmd = 0x00;
	hts221_set_reg(HTS221_CTRL_REG3,cmd);

}

/* HTS221����һ��ת�� */
static void HTS221_Start()
{
	unsigned char dat = 0;

	//��ȡREG2�Ĵ����е�ֵ����ֹ������Ϣ���ƻ�
	hts11_get_reg(HTS221_CTRL_REG2, &dat);

	//����һ��ת��
	dat |= 0x01;
	hts221_set_reg(HTS221_CTRL_REG2,dat);

}

/* ����һ��ת������ȡУ�����¶�ֵ */
/* note����API��ȡ��ֵ��10������   */
unsigned char	HTS221_Get_Temperature(int16_t* temperature)
{
	unsigned char status_dat = 0;
	signed short T0_degC, T1_degC;
	signed short T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	unsigned char T0_degC_x8, T1_degC_x8, tmp;
	unsigned char buffer[4];
	int32_t tmp32;

	/*1. ��ȡT0_degC_x8 �� T1_degC_x8 У��ֵ */
	hts11_get_reg(0x32,&T0_degC_x8);
	hts11_get_reg(0x33,&T1_degC_x8);

	/*2. ��ȡT1_degC �� T0_degC �����λ*/
	hts11_get_reg(0x35, &tmp);

	// ����T0_degC and T1_degC ֵ */
	T0_degC_x8_u16 = (((unsigned short)(tmp & 0x03)) << 8) | ((unsigned short)T0_degC_x8);
	T1_degC_x8_u16 = (((unsigned short)(tmp & 0x0C)) << 6) | ((unsigned short)T1_degC_x8);
	T0_degC = T0_degC_x8_u16>>3;
	T1_degC = T1_degC_x8_u16>>3;

	/*3. ��ȡ T0_OUT �� T1_OUT ֵ */
	hts11_get_reg(0x3C,&buffer[0]);
	hts11_get_reg(0x3D,&buffer[1]);
	hts11_get_reg(0x3E,&buffer[2]);
	hts11_get_reg(0x3F,&buffer[3]);

	T0_out = (((unsigned short)buffer[1])<<8) | (unsigned short)buffer[0];
	T1_out = (((unsigned short)buffer[3])<<8) | (unsigned short)buffer[2];

	/* 4. ����ת�����ȴ���ɺ��ȡת�����ֵT_OUT */
	HTS221_Start();
	while(status_dat != 0x03)
	{
		hts11_get_reg(HTS221_STATUS_REG, &status_dat);
	}
	hts11_get_reg(HTS221_TEMP_OUT_L, &buffer[0]);
	hts11_get_reg(HTS221_TEMP_OUT_H, &buffer[1]);

	T_out = (((unsigned short)buffer[1])<<8) | (unsigned short)buffer[0];

	/* 5. ʹ�����Բ�ֵ�����㵱ǰ��Ӧ���¶�ֵ */
	tmp32 = ((int32_t)(T_out - T0_out)) * ((int32_t)(T1_degC - T0_degC)*10);
	*temperature = tmp32 /(T1_out - T0_out) + T0_degC*10;

	return 0;
}

/* ����һ��ת������ȡУ����ʪ��ֵ */
/* note����API��ȡ��ֵ��10������   */
unsigned char	HTS221_Get_Humidity(signed short* humidity)
{
	unsigned char status_dat = 0;
	signed short H0_T0_out, H1_T0_out, H_T_out;
	signed short H0_rh, H1_rh;
	unsigned char buffer[2];
	int tmp;


	/* 1. ��ȡH0_rH and H1_rH У��ֵ */
	hts11_get_reg(0x30,&buffer[0]);
	hts11_get_reg(0x31,&buffer[1]);
	H0_rh = buffer[0] >> 1;
	H1_rh = buffer[1] >> 1;

	/*2. ��ȡ H0_T0_OUT У��ֵ */
	hts11_get_reg(0x36,&buffer[0]);
	hts11_get_reg(0x37,&buffer[1]);
	H0_T0_out = (((unsigned short)buffer[1])<<8) | (unsigned short)buffer[0];

	/*3. ��ȡ H1_T0_OUT У��ֵ */
	hts11_get_reg(0x3A,&buffer[0]);
	hts11_get_reg(0x3B,&buffer[1]);
	H1_T0_out = (((unsigned short)buffer[1])<<8) | (unsigned short)buffer[0];

	/*4. ����ת�����ȴ���ɺ��ȡת�����ֵ */
	HTS221_Start();
	while(status_dat != 0x03)
	{
		hts11_get_reg(HTS221_STATUS_REG, &status_dat);
	}

	hts11_get_reg(HTS221_HUMIDITY_OUT_L, &buffer[0]);
	hts11_get_reg(HTS221_HUMIDITY_OUT_H, &buffer[1]);
	H_T_out = (((unsigned short)buffer[1])<<8) | (unsigned short)buffer[0];

	/*5. ʹ�����Բ�ֵ������ʪ��ֵ RH [%] */
	tmp = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh)*10);
	*humidity = (tmp/(H1_T0_out - H0_T0_out) + H0_rh*10);
	//ʪ����ֵ�˲�
	if(*humidity>1000)
	{
		*humidity = 1000;
	}

	return 0;
}


void main(void)
{
	int16_t temperature, humidity;

	init_i2c();

	HTS221_Init();

	while(1){
		HTS221_Get_Temperature(&temperature);
		HTS221_Get_Humidity(&humidity);
		printf("T=%d,H=%d\r\n",temperature,humidity);
		sleep(1);
	}

	exit_i2c();
}