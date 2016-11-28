/*
 * defines.h
 *
 *  Created on: May 13, 2015
 *      Author: sara
 */

#ifndef DEFINES_H_
#define DEFINES_H_
//#include <sys\timeb.h>
#define CLOCK_PROCESS_CPUTIME_ID 2
#define getDiff(end,start,time) { timespec temp; if ((end.tv_nsec-start.tv_nsec)<0) {temp.tv_sec = end.tv_sec-start.tv_sec-1;temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;} else {temp.tv_sec = end.tv_sec-start.tv_sec;temp.tv_nsec = end.tv_nsec-start.tv_nsec;}time = temp.tv_nsec/1000000+temp.tv_sec*1000 ;}
//#define getDiff(end,start) (float) (1000000000.0 * (end.tv_sec-start.tv_sec-1) + (end.tv_nsec-start.tv_nsec));
#if defined(_WIN32) || defined(__MACH__)
#define timeOfBlock(block,time) {clock_t time1, time2; time1=clock(); block time2=clock();time = static_cast<double>(time2 - time1)/CLOCKS_PER_SEC; } //cerr<<#time<<" "<<time<<endl;}
#else
#define timeOfBlock(block,time) {timespec time1, time2; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1); block clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);getDiff(time2,time1,time); } //cerr<<#time<<" "<<time<<endl;}
#endif
//#define getDiff(end,start) (float) (1000.0 * (end.time - start.time) + (end.millitm - start.millitm));
//#define timeOfBlock(block,time) {struct timeb opstart, opend; ftime(&opstart); block ftime(&opend); time = getDiff(opend,opstart); /*cerr<<#time<<" "<<time<<endl;*/}


#endif /* DEFINES_H_ */
