#include "stdafx.h"

#include "../com.h"
#include "adcConstDlg.h"

LRESULT CALLBACK adcConstWndProc(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	LP_MAIN_DATA lpMainData = mainDataGet( GetParent(hDlg) );

	switch(message){
	case WM_INITDIALOG:
	 return TRUE;

	case WM_COMMAND:
		
		switch( LOWORD(wParam) ) {
		case IDCANCEL: EndDialog(hDlg, LOWORD(wParam)); break;

		case IDC_BUTTON_ADC0_SET_CONST: case IDC_BUTTON_ADC1_SET_CONST:
		case IDC_BUTTON_ADC2_SET_CONST: case IDC_BUTTON_ADC3_SET_CONST:
		case IDC_BUTTON_ADC4_SET_CONST: case IDC_BUTTON_ADC5_SET_CONST:
		case IDC_BUTTON_ADC6_SET_CONST: {
			if( !lpMainData->flagAdcConst ) {
				lpMainData->flagAdcConstId = LOWORD(wParam) - IDC_BUTTON_ADC0_SET_CONST;
				lpMainData->flagAdcConst = IDC_BUTTON_ADC0_SET_CONST;
				lpMainData->hwndAdcDialog = hDlg;
			}
		}
		 break;

		case IDC_BUTTON_ADC_READ_CONST: {
			if( !lpMainData->flagAdcConst ) {
				lpMainData->flagAdcConst = IDC_BUTTON_ADC_READ_CONST;
				lpMainData->hwndAdcDialog = hDlg;
			}
		}
		 break;

		case IDC_BUTTON_ADC_LOAD_DEF_CONST:
			if( !lpMainData->flagAdcConst ) {
				lpMainData->flagAdcConst = IDC_BUTTON_ADC_LOAD_DEF_CONST;
				lpMainData->hwndAdcDialog = hDlg;
			}
		 break;

		case IDC_BUTTON_ADC_LOAD_FROM_EEPROM:
			if( !lpMainData->flagAdcConst ) {
				lpMainData->flagAdcConst = IDC_BUTTON_ADC_LOAD_FROM_EEPROM;
				lpMainData->hwndAdcDialog = hDlg;
			}
		 break;

		case ID_ADC_CONST_SAVE_TO_EEPROM:
			if( !lpMainData->flagAdcConst ) {
				lpMainData->flagAdcConst = ID_ADC_CONST_SAVE_TO_EEPROM;
			}
		 break;
		}

	 return TRUE;
	}
    return FALSE;
}
