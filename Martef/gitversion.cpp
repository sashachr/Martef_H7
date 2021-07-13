/*
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2013 NM - Nanomotion Ltd.                           
 * All Rights Reserved.                                              
 * Licensed Material-Property of Nanomotion Ltd.                     
 * This software is made available solely pursuant to the terms      
 * of Nanomotion Ltd license agreement which  governs its use.       
 * This code and the information contained in it are  proprietary    
 * and confidential to Nanomotion Ltd.                               
 * No person is allowed to copy, reprint, reproduce or publish       
 * any part of this code, nor disclose its contents to others,       
 * nor make any use of it, nor allow or assist others to make        
 * any use of it unless by prior written express authorization of    
 * Nanomotion and then, only to the extent authorized.  
 ******************************************************************************
 *
 * @file    : GitVersion.cpp
 * @author  : Sasha
 * @version : xx.xx.xx
 * @date    : xx.xx.xxxx
 * @brief   : This file contain GIT version.
**/

#include "chip.h"
//#include "GitVersion.h"   // The file is generated in prebuild and deleted in postbuild

uint8_t GitVersion[] = {0}; //GIT_VERSION;
uint8_t GitSha[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //GIT_SHA;
