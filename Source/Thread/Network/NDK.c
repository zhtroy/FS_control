/*
 * NDK.c
 *
 *  Created on: 2018-12-12
 *      Author: zhtro
 */


/****************************************************************************/
/*                                                                          */
/*              广州创龙电子科技有限公司                                    */
/*                                                                          */
/*              Copyright (C) 2015 Tronlong Electronic Technology Co.,Ltd   */
/*                                                                          */
/****************************************************************************/
// C 语言函数库
#include <stdio.h>
#include <string.h>

// NDK
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>

#include <ti/ndk/inc/nettools/inc/configif.h>

#include <ti/ndk/inc/tools/console.h>
#include <ti/ndk/inc/tools/servers.h>

// SYS/BIOS
#include <ti/sysbios/knl/task.h>

// 外设驱动库
#include "uartStdio.h"

#include "emifa/emifa_app.h"

#include "TL6748.h"

/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/
// NDK 运行标志
char NDKFlag;

// MAC 地址
unsigned char bMacAddr[8];

// 连接状态
char *LinkStr[] = {"No Link", "10Mb/s Half Duplex", "10Mb/s Full Duplex", "100Mb/s Half Duplex", "100Mb/s Full Duplex"};


// 字符串
char *VerStr = "\nTronlong TCP/IP NDK Application ......\r\n";

// 静态 IP 配置
char *HostName     = "Tronlong-DSP_C6748";
char *LocalIPAddr  = "0.0.0.0";          // DHCP 模式下设置为 "0.0.0.0"
char *StaticIPAddr = "10.0.5.2";         // 开发板默认静态 IP
char *LocalIPMask  = "255.255.255.0";    // DHCP 模式下无效
char *GatewayIP    = "10.0.5.1";         // DHCP 模式下无效
char *DomainName   = "x.51dsp.net";      // DHCP 模式下无效
// 114.114.114.114 国内公共 DNS 服务器
char *DNSServer    = "114.114.114.114";  // 使用时设置为非 0 值

// DHCP 选项
unsigned char DHCP_OPTIONS[] = {DHCPOPT_SERVER_IDENTIFIER, DHCPOPT_ROUTER};

// 句柄
HANDLE hEcho = 0, hEchoUdp = 0, hData = 0, hNull = 0, hOob = 0;
static HANDLE hTcp = 0;

/****************************************************************************/
/*                                                                          */
/*              函数声明                                                    */
/*                                                                          */
/****************************************************************************/
void NetworkOpen();
void NetworkClose();
void NetIPAddrChange(IPN IPAddr, unsigned int IfIdx, unsigned int fAdd);

void ServiceReport(unsigned int Item, unsigned int Status, unsigned int Report, HANDLE h);
void DHCPReset(unsigned int IfIdx, unsigned int fOwnTask);
void DHCPStatus();

void AddWebFiles();
void RemoveWebFiles();
void TaskNDKInit();

/****************************************************************************/
/*                                                                          */
/*              回调函数 EMAC 初始化                                        */
/*                                                                          */
/****************************************************************************/
// 这个函数被驱动调用 不要修改函数名
void EMAC_initialize()
{

	// 管脚复用配置
	// 使能 MII 模式
	EMACPinMuxSetup();
}

/****************************************************************************/
/*                                                                          */
/*              回调函数 获取 MAC 地址                                      */
/*                                                                          */
/****************************************************************************/
// 这个函数被驱动调用 不要修改函数名
void EMAC_getConfig(unsigned char *pMacAddr)
{
	// 根据芯片 ID 生成 MAC 地址
	bMacAddr[0] = 0x00;
	bMacAddr[1] = (*(volatile unsigned int *)(0x01C14008) & 0x0000FF00) >> 8;
	bMacAddr[2] = (*(volatile unsigned int *)(0x01C14008) & 0x000000FF) >> 0;
	bMacAddr[3] = (*(volatile unsigned int *)(0x01C1400C) & 0x0000FF00) >> 8;
	bMacAddr[4] = (*(volatile unsigned int *)(0x01C1400C) & 0x000000FF) >> 0;
	bMacAddr[5] = (*(volatile unsigned int *)(0x01C14010) & 0x000000FF) >> 0;
	UARTprintf("Using MAC Address: %02X-%02X-%02X-%02X-%02X-%02X\n",
    		bMacAddr[0], bMacAddr[1], bMacAddr[2], bMacAddr[3], bMacAddr[4], bMacAddr[5]);

    // 传递 MAC 地址
    mmCopy(pMacAddr, bMacAddr, 6);
}

/****************************************************************************/
/*                                                                          */
/*              回调函数 设置 MAC 地址                                      */
/*                                                                          */
/****************************************************************************/
// 这个函数被驱动调用 不要修改函数名
void EMAC_setConfig(unsigned char *pMacAddr)
{
    mmCopy(bMacAddr, pMacAddr, 6);
    UARTprintf(
            "Setting MAC Addr to: %02x-%02x-%02x-%02x-%02x-%02x\n",
            bMacAddr[0], bMacAddr[1], bMacAddr[2],
            bMacAddr[3], bMacAddr[4], bMacAddr[5]);
}

/****************************************************************************/
/*                                                                          */
/*              回调函数 获取连接状态                                       */
/*                                                                          */
/****************************************************************************/
// 这个函数被驱动调用 不要修改函数名
void EMAC_linkStatus(unsigned int phy, unsigned int linkStatus)
{
	UARTprintf("\r\nLink Status: %s on PHY %d\n", LinkStr[linkStatus], phy);

	if(!NDKFlag)
	{
		UARTPuts("Link Status has changed!Ready to restart NDK Stack!\n", -2);

		UARTPuts("Stoping ......\n", -2);
		NC_NetStop(0);
		UARTPuts("Starting ......\n", -2);
		TaskNDKInit();
	}

	NDKFlag = 0;
}

/****************************************************************************/
/*                                                                          */
/*              NDK 打开                                                    */
/*                                                                          */
/****************************************************************************/
void NetworkOpen()
{
    // 服务
    hEcho = TaskCreate(echosrv, "EchoSrv", OS_TASKPRINORM, 0x1400, 0, 0, 0);
    hData = TaskCreate(datasrv, "DataSrv", OS_TASKPRINORM, 0x1400, 0, 0, 0);
    hNull = TaskCreate(nullsrv, "NullSrv", OS_TASKPRINORM, 0x1400, 0, 0, 0);
    hOob  = TaskCreate(oobsrv,   "OobSrv", OS_TASKPRINORM, 0x1000, 0, 0, 0);
    hTcp = DaemonNew(SOCK_STREAMNC, 0, 1000, dtask_tcp_echo, OS_TASKPRINORM, OS_TASKSTKNORM, 0, 3);
}

/****************************************************************************/
/*                                                                          */
/*              NDK 关闭                                                    */
/*                                                                          */
/****************************************************************************/
void NetworkClose()
{
	// 关闭会话
    fdCloseSession(hOob);
    fdCloseSession(hNull);
    fdCloseSession(hData);
    fdCloseSession(hEcho);

    // 关闭控制台
    ConsoleClose();

    // 删除任务
    TaskSetPri(TaskSelf(), NC_PRIORITY_LOW);

    TaskDestroy(hOob);
    TaskDestroy(hNull);
    TaskDestroy(hData);
    TaskDestroy(hEcho);
    DaemonFree(hTcp);
}

/****************************************************************************/
/*                                                                          */
/*              IP 地址                                                     */
/*                                                                          */
/****************************************************************************/
void NetworkIPAddr(IPN IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
	if(fAdd)
	{
        UARTprintf("Network Added: ", IfIdx);
	}
	else
	{
		UARTprintf("Network Removed: ", IfIdx);
	}

	char StrIP[16];
	NtIPN2Str(IPAddr, StrIP);
	UARTprintf("%s\r\n", StrIP);
}

/****************************************************************************/
/*                                                                          */
/*              服务状态                                                    */
/*                                                                          */
/****************************************************************************/
char *TaskName[]  = {"Telnet", "HTTP", "NAT", "DHCPS", "DHCPC", "DNS"};
char *ReportStr[] = {"", "Running", "Updated", "Complete", "Fault"};
char *StatusStr[] = {"Disabled", "Waiting", "IPTerm", "Failed", "Enabled"};

void ServiceReport(unsigned int Item, unsigned int Status, unsigned int Report, HANDLE h)
{
	UARTprintf("Service Status: %9s: %9s: %9s: %03d\n",
			     TaskName[Item - 1], StatusStr[Status], ReportStr[Report / 256], Report & 0xFF);

    // 配置 DHCP
    if(Item == CFGITEM_SERVICE_DHCPCLIENT &&
       Status == CIS_SRV_STATUS_ENABLED &&
       (Report == (NETTOOLS_STAT_RUNNING|DHCPCODE_IPADD) ||
        Report == (NETTOOLS_STAT_RUNNING|DHCPCODE_IPRENEW)))
    {
        IPN IPTmp;

        // 配置 DNS
        IPTmp = inet_addr(DNSServer);
        if(IPTmp)
        {
            CfgAddEntry(0, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER, 0, sizeof(IPTmp), (UINT8 *)&IPTmp, 0);
        }

        // DHCP 状态
        DHCPStatus();
    }

    // 重置 DHCP 客户端服务
    if(Item == CFGITEM_SERVICE_DHCPCLIENT && (Report & ~0xFF) == NETTOOLS_STAT_FAULT)
    {
        CI_SERVICE_DHCPC dhcpc;
        int tmp;

        // 取得 DHCP 入口数据(传递到 DHCP_reset 索引)
        tmp = sizeof(dhcpc);
        CfgEntryGetData(h, &tmp, (UINT8 *)&dhcpc);

        // 创建 DHCP 复位任务（当前函数是在回调函数中执行所以不能直接调用该函数）
        TaskCreate(DHCPReset, "DHCPreset", OS_TASKPRINORM, 0x1000, dhcpc.cisargs.IfIdx, 1, 0);
    }
}

/****************************************************************************/
/*                                                                          */
/*              DHCP 重置                                                   */
/*                                                                          */
/****************************************************************************/
void DHCPReset(unsigned int IfIdx, unsigned int fOwnTask)
{
    CI_SERVICE_DHCPC dhcpc;
    HANDLE h;
    int rc, tmp;
    unsigned int idx;

    // 如果在新创建的任务线程中被调用
    // 等待实例创建完成
    if(fOwnTask)
    {
        TaskSleep(500);
    }

    // 在接口上检索 DHCP 服务
    for(idx = 1; ; idx++)
    {
        // 寻找 DHCP 实例
        rc = CfgGetEntry(0, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, idx, &h);
        if(rc != 1)
        {
            goto RESET_EXIT;
        }

        // 取得 DHCP 实例数据
        tmp = sizeof(dhcpc);
        rc = CfgEntryGetData(h, &tmp, (UINT8 *)&dhcpc);

        if((rc<=0) || dhcpc.cisargs.IfIdx != IfIdx)
        {
            CfgEntryDeRef(h);
            h = 0;

            continue;
        }

        // 移除当前 DHCP 服务
        CfgRemoveEntry(0, h);

        // 在当前接口配置 DHCP 服务
        bzero(&dhcpc, sizeof(dhcpc));
        dhcpc.cisargs.Mode   = CIS_FLG_IFIDXVALID;
        dhcpc.cisargs.IfIdx  = IfIdx;
        dhcpc.cisargs.pCbSrv = &ServiceReport;
        CfgAddEntry(0, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, 0, sizeof(dhcpc), (UINT8 *)&dhcpc, 0);

        break;
    }

RESET_EXIT:
    if(fOwnTask)
    {
        TaskExit();
    }
}

/****************************************************************************/
/*                                                                          */
/*              DHCP 状态                                                   */
/*                                                                          */
/****************************************************************************/
void DHCPStatus()
{
    char IPString[16];
    IPN IPAddr;
    int i, rc;

    // 扫描 DHCPOPT_SERVER_IDENTIFIER 服务
    UARTprintf("\nDHCP Server ID:\n");
    for(i = 1; ; i++)
    {
        // 获取 DNS 服务
        rc = CfgGetImmediate(0, CFGTAG_SYSINFO, DHCPOPT_SERVER_IDENTIFIER, i, 4, (UINT8 *)&IPAddr);
        if(rc != 4)
        {
            break;
        }

        // IP 地址
        NtIPN2Str(IPAddr, IPString);
        UARTprintf("DHCP Server %d = '%s'\n", i, IPString);
    }

    if(i == 1)
    {
    	UARTprintf("None\n\n");
    }
    else
    {
    	UARTprintf("\n");
    }

    // 扫描 DHCPOPT_ROUTER 服务
    UARTprintf("Router Information:\n");
    for(i = 1; ; i++)
    {
        // 获取 DNS 服务
        rc = CfgGetImmediate(0, CFGTAG_SYSINFO, DHCPOPT_ROUTER, i, 4, (UINT8 *)&IPAddr);
        if(rc != 4)
        {
            break;
        }

        // IP 地址
        NtIPN2Str(IPAddr, IPString);
        UARTprintf("Router %d = '%s'\n", i, IPString);
    }

    if(i == 1)
    {
    	UARTprintf("None\n\n");
    }
    else
    {
    	UARTprintf("\n");
    }
}

/****************************************************************************/
/*                                                                          */
/*              任务（Task）线程                                            */
/*                                                                          */
/****************************************************************************/
Void NDKTask(UArg a0, UArg a1)
{
    int rc;

    // 初始化操作系统环境
    // 必须在使用 NDK 之前最先调用
    rc = NC_SystemOpen(NC_PRIORITY_HIGH, NC_OPMODE_INTERRUPT);
    if(rc)
    {
    	UARTprintf("NC_SystemOpen Failed (%d)\n", rc);

        for(;;);
    }

    // 创建新的配置
    HANDLE hCfg;
    hCfg = CfgNew();
    if(!hCfg)
    {
    	UARTprintf("Unable to create configuration\n");

        goto Exit;
    }

    // 配置主机名
    if(strlen( DomainName ) >= CFG_DOMAIN_MAX || strlen( HostName ) >= CFG_HOSTNAME_MAX)
    {
    	UARTprintf("Names too long\n");

        goto Exit;
    }

    // 添加全局主机名到 hCfg(对所有连接域有效)
    CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_HOSTNAME, 0, strlen(HostName), (UINT8 *)HostName, 0);

    // 静态或动态 IP 配置
    char flag;
    UARTprintf("Use static IP or get IP via DHCP Server?\n");
    UARTprintf("Please input Y or N, Y - Static IP N - DHCP\n\n");
    flag = UARTGetc();
    if(flag == 'Y' ||  flag == 'y')
    {
    	char IPAddr[16];
        UARTprintf("\r\nInput IP address(Example 10.0.5.2)\n");
        UARTGets(IPAddr, -2);

    	char IPMask[16];
        UARTprintf("\r\nInput subnet mask(Example 255.255.255.0)\n");
        UARTGets(IPMask, -2);

    	char IPGateway[16];
        UARTprintf("\r\nInput default gateway(Example 10.0.5.1)\n");
        UARTGets(IPGateway, -2);

		CI_IPNET NA;
		CI_ROUTE RT;
		IPN      IPTmp;

		// 配置 IP
		bzero(&NA, sizeof(NA));
		NA.IPAddr = inet_addr(IPAddr);
		NA.IPMask = inet_addr(IPMask);
		strcpy(NA.Domain, DomainName);
		NA.NetType = 0;

		// 添加地址到接口 1
		CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0, sizeof(CI_IPNET), (UINT8 *)&NA, 0);

		// 配置 默认网关
		bzero(&RT, sizeof(RT));
		RT.IPDestAddr = 0;
		RT.IPDestMask = 0;
		RT.IPGateAddr = inet_addr(IPGateway);

		// 配置 路由
		CfgAddEntry(hCfg, CFGTAG_ROUTE, 0, 0, sizeof(CI_ROUTE), (UINT8 *)&RT, 0);

		// 配置 DNS 服务器
		IPTmp = inet_addr(DNSServer);
		if(IPTmp)
		CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER, 0, sizeof(IPTmp), (UINT8 *)&IPTmp, 0);

		// 输出配置信息
        UARTprintf("\r\n\r\nIP Address is set to %s\n", IPAddr);
        UARTprintf("IP subnet mask is set to %s\n", IPMask);
        UARTprintf("IP default gateway is set to %s\r\n\r\n", IPGateway);

    }
    else if(flag == 'N' ||  flag == 'n')
    {
        CI_SERVICE_DHCPC dhcpc;

        // 指定 DHCP 服务运行在接口 IF-1
        bzero(&dhcpc, sizeof(dhcpc));
        dhcpc.cisargs.Mode   = CIS_FLG_IFIDXVALID;
        dhcpc.cisargs.IfIdx  = 1;
        dhcpc.cisargs.pCbSrv = &ServiceReport;
        dhcpc.param.pOptions = DHCP_OPTIONS;
        dhcpc.param.len = 2;
        CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, 0, sizeof(dhcpc), (UINT8 *)&dhcpc, 0);
    }
    else
    {
    	UARTprintf("Please input Y or N, Y - Static IP N - DHCP\n");
    	flag = UARTGetc();
    }

    // 配置 TELNET 服务
    CI_SERVICE_TELNET telnet;

    bzero(&telnet, sizeof(telnet));
    telnet.cisargs.IPAddr = INADDR_ANY;
    telnet.cisargs.pCbSrv = &ServiceReport;
    telnet.param.MaxCon   = 2;
    telnet.param.Callback = &ConsoleOpen;
    CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_TELNET, 0, sizeof(telnet), (UINT8 *)&telnet, 0);

    // 配置 HTTP 相关服务
    // 添加静态网页文件到 RAM EFS 文件系统
//    AddWebFiles();

    // HTTP 身份认证
	CI_ACCT CA;

	// 命名 HTTP 服务身份认证组(最大长度 31)
	CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_SYSINFO_REALM1, 0, 30, (UINT8 *)"DSP_CLIENT_AUTHENTICATE1", 0 );

	// 创建用户
	// 用户名 密码均为 admin
	strcpy(CA.Username, "admin");
	strcpy(CA.Password, "admin");
	CA.Flags = CFG_ACCTFLG_CH1;  // 成为 realm 1 成员
	rc = CfgAddEntry(hCfg, CFGTAG_ACCT, CFGITEM_ACCT_REALM, 0, sizeof(CI_ACCT), (UINT8 *)&CA, 0);

	// 配置 HTTP 服务
    CI_SERVICE_HTTP http;
    bzero(&http, sizeof(http));
    http.cisargs.IPAddr = INADDR_ANY;
    http.cisargs.pCbSrv = &ServiceReport;
    CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_HTTP, 0, sizeof(http), (UINT8 *)&http, 0);

    // 配置协议栈选项

    // 显示警告消息
    rc = DBG_INFO;
    CfgAddEntry(hCfg, CFGTAG_OS, CFGITEM_OS_DBGPRINTLEVEL, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // 设置 TCP 和 UDP buffer 大小（默认 8192）
    // TCP 发送 buffer 大小
    rc = 8192;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPTXBUF, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // TCP 接收 buffer 大小(copy 模式)
    rc = 8192;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXBUF, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // TCP 接收限制大小(non-copy 模式)
    rc = 8192;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXLIMIT, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // UDP 接收限制大小
    rc = 8192;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKUDPRXLIMIT, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

#if 0
    // TCP Keep Idle(10 秒)
    rc = 100;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_TCPKEEPIDLE, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // TCP Keep Interval(1 秒)
    rc = 10;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_TCPKEEPINTVL, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);

    // TCP Max Keep Idle(5 秒)
    rc = 50;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_TCPKEEPMAXIDLE, CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0);
#endif

    // 使用当前配置启动 NDK 网络
    NDKFlag = 1;
    do
    {
        rc = NC_NetStart(hCfg, NetworkOpen, NetworkClose, NetworkIPAddr);
    } while(rc > 0);

    // 停止消息
    UARTprintf("NDK Task has been stop(Return Code %d)!\r\n", rc);

    // 移除 WEB 文件
//    RemoveWebFiles();

    // 删除配置
    CfgFree(hCfg);

    // 退出
    goto Exit;

Exit:
    NC_SystemClose();

    TaskExit();
}

/****************************************************************************/
/*                                                                          */
/*              任务（Task）线程初始化                                      */
/*                                                                          */
/****************************************************************************/
void TaskNDKInit()
{
	//复位PHY芯片 R127
//	EMIFA_write(0x30, 1);

    // NDK 任务
	Task_Handle NDKTaskHandle;
	Task_Params TaskParams;

	Task_Params_init(&TaskParams);
	TaskParams.stackSize = 512 * 1024;
	TaskParams.priority = 5;

	NDKTaskHandle = Task_create(NDKTask, &TaskParams, NULL);
	if(NDKTaskHandle == NULL)
	{
		UARTprintf("NDK Task create failed!\r\n");
	}
}
