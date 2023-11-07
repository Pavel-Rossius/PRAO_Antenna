/*
 * Copyright (c) 2020 NVI, Inc.
 *
 * This file is part of VLBI Field System
 * (see http://github.com/nvi-inc/fs).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* antcn.c
 *
 * This is the stub version of antcn (ANTenna CoNtrol program).
 * This version sends a log message whenever it is called.
 *
 */

/**********************************************************************
*                                                                     *
* PRAO RT-22 ANTENNA CLIENT MODIFICATION by Pavel Rossius @ 07/2021   *
*                                                                     *
**********************************************************************/

/* Input */
/* IP(1) = mode
       0 = initialize LU
       1 = pointing (from SOURCE command)
       2 = offset (from RADECOFF, AZELOFF, or XYOFF commands)
       3 = on/off source status (from ONSOURCE command)
       4 = direct communications (from ANTENNA command)
       5 = on/off source status for pointing programs
       6 = reserved for future focus control
       7 = log tracking data (from TRACK command)
       8 = Station detectors, see /usr2/fs/misc/stndet.txt
       9 = Satellite traking, see /usr2/fs/misc/satellites.txt
      10 = termination mode, must return promptly
 11 - 99 = reserved for future use
100 - 32767 = for site specific use

   IP(2) = class number (mode 4 only)
   IP(3) = number of records in class (mode 4 only)
   IP(4) - not used
   IP(5) - not used
*/

/* Output */
/*  IP(1) = class with returned message
      (2) = number of records in class
      (3) = error number
            0 - ok
           -1 - illegal mode
           -2 - timeout
           -3 - wrong number of characters in response
           -4 - interface not set to remote
           -5 - error return from antenna
           -6 - error in pointing model initialization
            others as defined locally
      (4) = 2HAN for above errors, found in FSERR.CTL
          = 2HST for site defined errors, found in STERR.CTL
      (5) = not used
*/

/* Defined variables */
#define MINMODE 0  /* min,max modes for our operation */
#define MAXMODE 10

/* Include files */

#include "../include/params.h" /* FS parameters            */
#include "../include/fs_types.h" /* FS header files        */
#include "../include/fscom.h"  /* FS shared mem. structure */
#include "../include/shm_addr.h" /* FS shared mem. pointer */

/* PRAO RT22 TCP-Socket Stuff */

#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>

#include <sys/socket.h>
#include <netinet/in.h> //for 'sockaddr_in' structure
#include <arpa/inet.h> //for inet_addr() & htons()

#include <unistd.h> //for close()
#include <string.h> //for strlen()

#include <math.h> //for math

struct fscom *fs;

char * rt22msg(char * msg)
{
  static char bf[512] = {0};
  memset(bf, 0 , 512);
  int sock;
  struct sockaddr_in addr;
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    memset(bf, 0 , 512);
    strcpy(bf, "WARNING: Unable to create a Socket!\n");
    return bf;
  }
  else
  {
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5001);
    addr.sin_addr.s_addr = inet_addr("192.168.0.161");
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      memset(bf, 0 , 512);
      strcpy(bf, "WARNING: Socket connection error!\n");
      return bf;
    }
    else
    {
      // Sending message
      if (send(sock, msg, strlen(msg), 0) < 0)
      {
        memset(bf, 0 , 512);
        strcpy(bf, "WARNING: Sending failed!\n");
        return bf;
      }
      else
      {
        // Receiving message
        if (recv(sock, bf, 512, 0) < 0)
        {
          memset(bf, 0 , 512);
          strcpy(bf, "WARNING: Receiving failed!\n");
          return bf;
        }
        else
        {
          //Everything is OK
          bf[strlen(bf) - 1] = 0;
        }
      }
    }
    close(sock);
  }
  return bf;
}

/* Subroutines called */
void setup_ids();
void putpname();
void skd_run(), cls_clr();
int nsem_test();
void logit();

/* antcn main program starts here */
main()
{
  int ierr, nrec, nrecr;
  int dum = 0;
  int r1, r2;
  int imode,i,nchar;
  int ip[5], class, clasr;
  char buf[80], buf2[100];

/* Set up IDs for shared memory, then assign the pointer to
   "fs", for readability.
*/
  setup_ids();
  fs = shm_addr;

/* Put our program name where logit can find it. */

  putpname("antcn");

/* Return to this point to wait until we are called again */

Continue:
  skd_wait("antcn",ip,(unsigned)0);

  imode = ip[0];
  class = ip[1];
  nrec = ip[2];

  nrecr = 0;
  clasr = 0;

  if (imode < MINMODE || imode > MAXMODE) {
    ierr = -1;
    goto End;
  }

/* Handle each mode in a separate section */

  switch (imode) {

    case 0: /* initialize */
      ierr = 0;
      strcpy(buf, "Initializing PRAO RT-22 Antenna Interface...");
      logit(buf, 0, NULL);

      char conntest[512] = {0};

      strcpy(buf, "Sending message to RT-22 Antenna...");
      logit(buf, 0, NULL);
      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg("SEND_MESSG RT-22 Antenna Field System Client Is Connected.\r\n"));
      logit(conntest, 0, NULL);

      strcpy(buf, "GET_COOCAL : Alpha, Delta, F_Error, A1, H1");
      logit(buf, 0, NULL);
      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg("GET_COOCAL\r\n"));
      logit(conntest, 0, NULL);

      strcpy(buf, "GET_COOCUR : Alpha_Cur, Delta_Cur, F_Error, Er_CurA, Er_CurH");
      logit(buf, 0, NULL);
      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg("GET_COOCUR\r\n"));
      logit(conntest, 0, NULL);

      strcpy(buf, "GET_METEOD : T, P, WL");
      logit(buf, 0, NULL);
      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg("GET_METEOD\r\n"));
      logit(conntest, 0, NULL);
      
      // SET RUPORS TO RT-22 VLBI STANDART 

      double fl_a = -(580 * M_PI) / 648000;
      char cfl_a[32] = {0}; 
      snprintf(cfl_a, 32, "%.32f", fl_a);
      
      double fl_h = (530 * M_PI) / 648000;
      char cfl_h[32] = {0}; 
      snprintf(cfl_h, 32, "%.32f", fl_h);

      double fr_a = (580 * M_PI) / 648000;
      char cfr_a[32] = {0}; 
      snprintf(cfr_a, 32, "%.32f", fr_a);

      double fr_h = (530 * M_PI) / 648000;
      char cfr_h[32] = {0}; 
      snprintf(cfr_h, 32, "%.32f", fr_h);

      int ilrb = -1;
      char cilrb[3] = {0}; 
      snprintf(cilrb, 3, "%d", ilrb);

      double rerror = (40 * M_PI) / 648000;
      char crerror[32] = {0}; 
      snprintf(crerror, 32, "%.32f", rerror);
      
      char csetr[32] = {0};
      strcpy(csetr, "SET_RUPORS ");

      char csp[2] = {0};
      strcpy(csp, " ");

      char cend[8] = {0};
      strcpy(cend, "\r\n");

      char crcom[512] = {0};
      strcat(crcom, csetr);   //"SET RUPORS "
      strcat(crcom, cfl_a);   //"SET RUPORS CFL_A"
      strcat(crcom, csp);     //"SET RUPORS CFL_A "
      strcat(crcom, cfl_h);   //"SET RUPORS CFL_A CFL_H"
      strcat(crcom, csp);     //"SET RUPORS CFL_A CFL_H "
      strcat(crcom, cfr_a);   //"SET RUPORS CFL_A CFL_H CFR_A"
      strcat(crcom, csp);     //"SET RUPORS CFL_A CFL_H CFR_A "
      strcat(crcom, cfr_h);   //"SET RUPORS CFL_A CFL_H CFR_A CFR_H"
      strcat(crcom, csp);     //"SET RUPORS CFL_A CFL_H CFR_A CFR_H "
      strcat(crcom, cilrb);   //"SET RUPORS CFL_A CFL_H CFR_A CFR_H CILRB"
      strcat(crcom, csp);     //"SET RUPORS CFL_A CFL_H CFR_A CFR_H CILRB "
      strcat(crcom, crerror); //"SET RUPORS CFL_A CFL_H CFR_A CFR_H CILRB CRERROR"
      strcat(crcom, cend);    //"SET RUPORS CFL_A CFL_H CFR_A CFR_H CILRB CRERROR\r\n"

      strcpy(buf, "Setting RT-22 Antenna Rupors To Default Values For VLBI Experiment...");
      logit(buf, 0, NULL);

      logit(crcom, 0, NULL);
      rt22msg(crcom);

      strcpy(buf, "GET_RUPORS : FL_A, FL_H, FR_A, FR_H, ILRB, Error");
      logit(buf, 0, NULL);
      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg("GET_RUPORS\r\n"));
      logit(conntest, 0, NULL);

      fs->ionsor = 0;
      break;
      
    case 1: /* source=command */
      ierr = 0;
      strcpy(buf, "Commanding to a new source");
      logit(buf, 0, NULL);
      
      char rtsrc[512] = {0};
      char st0[32] = {0};
      char st1[32] = {0}; //char st2[32]; char st3[32]; char st4[32]; char st5[32]; char st6[32];
      char snm[10] = {0};
      strcpy(st0, "SEND_MESSG New Source : ");
      strncpy(snm, fs->lsorna, sizeof(snm) - 1);
      snm[sizeof(snm) - 1] = '\0';
      strcpy(st1, "\r\n");
      strcat(snm, st1);
      strcat(st0, snm);
      logit("Generated String :", 0, NULL);
      logit(st0, 0, NULL);
      
      logit("Answer From RT22MSG Function :", 0, NULL);
      strcpy(rtsrc, rt22msg(st0));
      logit(rtsrc, 0, NULL);
      
      /*Some variables from source*/
      /*                                      strcpy(st0, "fs->lsorna : "); strcat(st0, snm); logit(st0, 0, NULL);
      snprintf(st2, 32, "%.32f", fs->ra50);   strcpy(st0, "fs->ra50   : "); strcat(st0, st2); logit(st0, 0, NULL);
      snprintf(st3, 32, "%.32f", fs->dec50);  strcpy(st0, "fs->dec50  : "); strcat(st0, st3); logit(st0, 0, NULL);
      snprintf(st4, 32, "%.32f", fs->ep1950); strcpy(st0, "fs->ep1950 : "); strcat(st0, st4); logit(st0, 0, NULL);
      snprintf(st5, 32, "%.32f", fs->radat);  strcpy(st0, "fs->radat  : "); strcat(st0, st5); logit(st0, 0, NULL);
      snprintf(st6, 32, "%.32f", fs->decdat); strcpy(st0, "fs->decdat : "); strcat(st0, st6); logit(st0, 0, NULL);
      */
      
      // SENDING NEW SOURCE COMMAND TO RT-22 ANTENNA

      char nsend[32] = {0};
      strcpy(nsend, " 1 0 0 0 \r\n"); //EPOCH DA DDEL PM \r\n

      char nsa0[32] = {0}; 
      snprintf(nsa0, 32, "%.32f", fs->ra50);  // ALPHA-0
      
      char nsd0[32] = {0}; 
      snprintf(nsd0, 32, "%.32f", fs->dec50); // DELTA-0

      char nsstr[512] = {0};
      strcpy(nsstr, "NEW_SOURCE "); //"NEW SOURCE "
      strcat(nsstr, nsa0);          //"NEW SOURCE NSA0"
      strcat(nsstr, csp);           //"NEW SOURCE NSA0 "
      strcat(nsstr, nsd0);          //"NEW SOURCE NSA0 NSD0"
      strcat(nsstr, nsend);         //"NEW SOURCE NSA0 NSD0 1 0 0 0 \r\n"

      strcpy(buf, "Setting RT-22 Antenna To New Source...");
      logit(buf, 0, NULL);

      logit(nsstr, 0, NULL);
      //rt22msg(nsstr);

      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg(nsstr));
      logit(conntest, 0, NULL);

      fs->ionsor = 0;
      break;

    case 2:             /* offsets         */
      ierr = 0;
      strcpy(buf,"Commanding new offsets");
      logit(buf,0,NULL);

      // SENDING SHIFTS/OFFSETS TO RT-22 ANTENNA

      char ofcor_a[32] = {0};
      snprintf(ofcor_a, 32, "%.32f", fs->AZOFF);

      char ofcor_h[32] = {0};
      snprintf(ofcor_h, 32, "%.32f", fs->ELOFF);

      char ofstr[256] = {0};
      strcpy(ofstr, "SET_SHIFTS "); //"SET_SHIFTS "
      strcat(ofstr, ofcor_a);       //"SET_SHIFTS ofcor_a"
      strcat(ofstr, csp);           //"SET_SHIFTS ofcor_a "
      strcat(ofstr, ofcor_h);       //"SET_SHIFTS ofcor_a ofcor_h"
      strcat(ofstr, cend);          //"SET_SHIFTS ofcor_a ofcor_h\r\n"

      strcpy(buf, "Setting RT-22 Antenna offsets...");
      logit(buf, 0, NULL);

      logit(ofstr, 0, NULL);
      //rt22msg(ofstr);

      memset(conntest, 0 , 512);
      strcpy(conntest, rt22msg(ofstr));
      logit(conntest, 0, NULL);

      fs->ionsor = 0;
      break;

    case 3:        /* onsource command with error message */
      ierr = 0;
      strcpy(buf,"Checking onsource status, extended error logging");
      logit(buf,0,NULL);
      fs->ionsor = 1;
      break;

    case 4:            /* direct antenna= command */
      if (class == 0)
        goto End;
      for (i=0; i<nrec; i++) {
        strcpy(buf2,"Received message for antenna: ");
        nchar = cls_rcv(class,buf,sizeof(buf),&r1,&r2,dum,dum);
        buf[nchar] = 0;  /* make into a string */
        strcat(buf2,buf);
        logit(buf2,0,NULL);
        strcpy(buf,"ACK");
        cls_snd(&clasr,buf,3,dum,dum);
        nrecr += 1;
      }
      /* OR:
         cls_clr(class);
         */
      break;

    case 5:    /* onsource command with no error logging */
      ierr = 0;
      strcpy(buf,"Checking onsource status, no error logging");
      logit(buf,0,NULL);
      fs->ionsor = 1;
      break;

    case 6:            /* reserved */
      ierr = -1;
      strcpy(buf,"TBD focus control");
      logit(buf,0,NULL);
      goto End;
      break;

    case 7:    /* onsource command with additional info  */
      ierr = 0;
      strcpy(buf,"Checking onsource status, log tracking data");
      logit(buf,0,NULL);
      fs->ionsor = 1;
      break;

  case 8:
      ierr = 0;
      strcpy(buf,"Station dependent detectors access");
      logit(buf,0,NULL);
      break;

  case 9:
      ierr = 0;
      strcpy(buf,"Satellite tracking mode");
      logit(buf,0,NULL);
      break;

  case 10: /*normally triggered on FS termination if environment variable
	     FS_ANTCN_TERMINATION has been defined */
      ierr = 0;
      strcpy(buf,"Termination mode");
      logit(buf,0,NULL);
      break;

  default:
      ierr = -1;
      strcpy(buf,"Impossible to reach");
      logit(buf,0,NULL);
      break;
  }  /* end of switch */

End:
  ip[0] = clasr;
  ip[1] = nrecr;
  ip[2] = ierr;
  memcpy(ip+3,"AN",2);
  ip[4] = 0;
  goto Continue;

}

