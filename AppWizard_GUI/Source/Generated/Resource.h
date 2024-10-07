/*********************************************************************
*                     SEGGER Microcontroller GmbH                    *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2024  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************
----------------------------------------------------------------------
File        : Resource.h
Purpose     : Generated file do NOT edit!
---------------------------END-OF-HEADER------------------------------
*/

#ifndef RESOURCE_H
#define RESOURCE_H

#include "AppWizard.h"

/*********************************************************************
*
*       Text
*/
#define ID_RTEXT_0 0
#define ID_RTEXT_1 1
#define ID_RTEXT_2 2
#define ID_RTEXT_3 3
#define ID_RTEXT_4 4

#define APPW_MANAGE_TEXT APPW_MANAGE_TEXT_EXT
extern GUI_CONST_STORAGE unsigned char acAPPW_Language_0[];

/*********************************************************************
*
*       Fonts
*/
extern GUI_CONST_STORAGE unsigned char acArial_12_Normal_EXT_AA4[];
extern GUI_CONST_STORAGE unsigned char acArial_14_Bold_EXT_AA4[];

/*********************************************************************
*
*       Images
*/
extern GUI_CONST_STORAGE unsigned char acpngwing_com[];
extern GUI_CONST_STORAGE unsigned char acthermometer[];

/*********************************************************************
*
*       Screens
*/
#define ID_SCREEN_00 (GUI_ID_USER + 4096)

extern APPW_ROOT_INFO ID_SCREEN_00_RootInfo;

#define APPW_INITIAL_SCREEN &ID_SCREEN_00_RootInfo

/*********************************************************************
*
*       Project path
*/
#define APPW_PROJECT_PATH "../../../AppWizard_GUI"

#endif  // RESOURCE_H

/*************************** End of file ****************************/
