/****************************************************************************
**
** Copyright (C) 2011 Beken Corporation, All rights reserved.
**
** Contact: Beken Corporation (www.beken.com)
**
** Author:  river
**
** History: 2012/03/07 
**
** Version: 1.0
**
****************************************************************************/
/*! \file application_rf.h
    \brief The application headfile of rf module.
*/

#ifndef _APPLICATION_RF_H_
#define _APPLICATION_RF_H_

void application_dongle_rf_receive_with_multi_hop_in_8ms(void);
void application_dongle_rf_cd_detect(UINT8 count);
void JumpFrequency(UINT8 step);		
void rf_pipe_set_public(void);

#endif

/***********************************************************
						end file
***********************************************************/
