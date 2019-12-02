/*
 * Tests.h
 *
 *  Created on: 2018-12-4
 *      Author: zhtro
 */

#ifndef TESTS_H_
#define TESTS_H_

void TestEntry();


void testStateMachine_init();
void testRFIDtask();
void testCantaskInit();
void testYAKINDU_SM_init();
void testSonicRadar_init();
void testUARTCommandLineInit();
void testWatchDogTaskInit();
void TaskNDKInit();
void testCellCom_init();
void testSimpleRun_init();
void testPhotoElectric_init();
void test4GControl_init();


void ServoTaskInit();
void MototaskInit();

void testMPU9250TaskInit(void);

void  test_array(void);
#endif /* TESTS_H_ */
