/*
 * 光电传感器
 *
 * 所有光电对管独占一条CAN总线
 *
 */
#include "canModule.h"
#include <xdc/runtime/Log.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <stdio.h>
#include "uartStdio.h"
#include "Sensor/PhotoElectric/PhotoElectric.h"
#include "Message/Message.h"

Semaphore_Handle g_sem_photoelectric;
static Mailbox_Handle rxDataMbox = NULL;



/********************************************************************************/
/*          外部CAN0的用户中断Handler                                                   */
/*                                                                              */
/********************************************************************************/
static void CAN0IntrHandler(int32_t devsNum,int32_t event)
{
    canDataObj_t rxData;
	/*
	 * a Frame has been received.
	 */
	if (event == 1) {
        //CurrntBuffer = canPushBuffer(devsNum);
        //canRead(devsNum, CurrntBuffer);
        //Semaphore_post(sem_photoelectric_dataReady);
        CanRead(devsNum, &rxData);
        Mailbox_post(rxDataMbox, (Ptr *)&rxData, BIOS_NO_WAIT);
	}

    if (event == 2) {
        /* 发送中断 */
    }

}

static void InitSem()
{


	Semaphore_Params semParams;
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	g_sem_photoelectric = Semaphore_create(0, &semParams, NULL);

    Mailbox_Params mboxParams;
    
    /* 初始化接收邮箱 */
    Mailbox_Params_init(&mboxParams);
    rxDataMbox = Mailbox_create (sizeof (canDataObj_t),PHOTO_MBOX_DEPTH, &mboxParams, NULL);
}

//解析并处理光电对管数据
static photo_t PhotoEleResolveCANdata(canDataObj_t * pdata)
{
	photo_data_t * pphoto;
	photo_t p;

	pphoto = (photo_data_t*) (pdata->Data);

	p.deviceNum = pdata->ID;
	memcpy(&(p.content), pphoto, sizeof(photo_data_t));

	return p;
}

void taskPhotoElectric()
{
    canDataObj_t canRecvData;
    photo_t photo;
    p_msg_t msg;
    /*
     * 对管状态，1为有光状态，0为无光
     */
    uint8_t levelState = 0;

    /*初始化信用量*/
    InitSem();
    /*初始化CAN设备*/
    CanOpen(PHOTO_CAN_DEV, CAN0IntrHandler, PHOTO_CAN_DEV);

    while(1)
    { 
        Mailbox_pend(rxDataMbox, (Ptr *) &canRecvData,BIOS_WAIT_FOREVER);

        photo = PhotoEleResolveCANdata(&canRecvData);

        /*
         * 串口输出，调试用
         */
        switch(levelState){
        case 0:
        	if(photo.content.cmd == CAN_CMD_RISE)
        	{
        		levelState = 1;
        	}
        	break;
        case 1:
        	if(photo.content.cmd == CAN_CMD_FALL )
        	{
        		msg = Message_getEmpty();
        		msg->type = photon;
        		Message_post(msg);

        		/*
        		 * 向刹车模块发信号
        		 * 这种跨模块之间共享信号量的方式不好，造成模块耦合
        		 */
        		Semaphore_post(g_sem_photoelectric);
        		levelState = 0;
        	}
        	break;
        }
        UARTprintf("photo deviceNum = %d\t",photo.deviceNum);
        if(photo.content.cmd ==CAN_CMD_RISE ){
			UARTPuts("rise: ",-1);
		}
		else if(photo.content.cmd == CAN_CMD_FALL){
			UARTPuts("fall: ",-1);
		}
		else{
			UARTPuts("unknown: ",-1);
		}

		UARTprintf("time: %d ms\tchn:%d\n", photo.content.data,photo.content.chn);

    }
}


/*
 *   API
 */

//FIXME: 目前arm板卡在上电后只能改一次CAN ID
void PhotoEleChangeID(uint32_t oldID, uint32_t newID)
{
	canDataObj_t can_obj;
	photo_data_t * pphoto;

	can_obj.ID = oldID;
	can_obj.SendType = 0;
	can_obj.DataLen = 8;

	pphoto = (photo_data_t *) (can_obj.Data);
	pphoto->cmd = CAN_CMD_ID;
	pphoto->chn  = 0;
	*(uint32_t*)(pphoto->data) = newID;

	CanWrite(PHOTO_CAN_DEV, &can_obj);
}

void PhotoEleSetLight(uint32_t id, uint8_t mask)
{
	canDataObj_t can_obj;
	photo_data_t * pphoto;

	can_obj.ID = id;
	can_obj.SendType = 0;
	can_obj.DataLen = 8;

	pphoto = (photo_data_t *) (can_obj.Data);
	pphoto->cmd = CAN_CMD_CTRL;
	pphoto->chn  = mask;

	CanWrite(PHOTO_CAN_DEV, &can_obj);

}

