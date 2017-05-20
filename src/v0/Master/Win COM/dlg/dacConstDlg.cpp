#include "stdafx.h"

#include "..\\com.h"
#include "adcConstDlg.h"

LRESULT CALLBACK dacConstWndProc(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	LP_MAIN_DATA lpMainData = mainDataGet( GetParent(hDlg) );

	switch(message){
	case WM_INITDIALOG:
	 return TRUE;

	case WM_COMMAND:
		switch(LOWORD(wParam)) {
		case IDCANCEL: EndDialog(hDlg, LOWORD(wParam)); break;

		case ID_DAC0_SET_CONST:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = ID_DAC0_SET_CONST;
				lpMainData->hwndDacDialog = hDlg;
			}
		 break;

		case ID_DAC1_SET_CONST:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = ID_DAC1_SET_CONST;
				lpMainData->hwndDacDialog = hDlg;
			}
		 break;

		case IDC_BUTTON_DAC_READ_CONST:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = IDC_BUTTON_DAC_READ_CONST;
				lpMainData->hwndDacDialog = hDlg;
			}
		 break;

		case IDC_BUTTON_DAC_LOAD_DEF_CONST:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = IDC_BUTTON_DAC_LOAD_DEF_CONST;
				lpMainData->hwndDacDialog = hDlg;
			}
		 break;
		case IDC_BUTTON_DAC_LOAD_FROM_EEPROM:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = IDC_BUTTON_DAC_LOAD_FROM_EEPROM;
				lpMainData->hwndDacDialog = hDlg;
			}
		 break;

		case ID_DAC_CONST_SAVE_TO_EEPROM:
			if( !lpMainData->flagDacConst ) {
				lpMainData->flagDacConst = ID_DAC_CONST_SAVE_TO_EEPROM;
			}
		 break;
		}

	 return TRUE;
	}
    return FALSE;
}
