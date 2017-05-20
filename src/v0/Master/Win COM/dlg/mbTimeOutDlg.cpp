#include "stdafx.h"

#include "..\\com.h"
#include "mbTimeOutDlg.h"

LRESULT CALLBACK mbTimeOutWndProc( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam )
{
	LP_MAIN_DATA lpMainData = mainDataGet( GetParent(hDlg) );
	char szBuffer[100];

	switch(message){
	case WM_INITDIALOG:
		lpMainData->hwndMbTimeOutDialog = hDlg;
	 return TRUE;

	case WM_COMMAND:
		switch( LOWORD(wParam) ) {
		case IDC_BUTTON_READ_MODBUS_TIMEOUT: {
			if( !lpMainData->uiFlagButton ) {
				lpMainData->uiFlagButton = IDC_BUTTON_READ_MODBUS_TIMEOUT;
			}
		}
		 break;

		/////////////////////////////////////////////////////////////////////////////////////////
		case IDC_BUTTON_WRITE_MODBUS_TIMEOUT: {
			if( !lpMainData->flagPidWrite ) {
				Edit_GetText( GetDlgItem( hDlg, IDC_EDIT_MODBUS_TIMEOUT ), szBuffer, 16 );
				lpMainData->uiMbTimeOut = atoi( szBuffer );
				lpMainData->uiFlagButton = IDC_BUTTON_WRITE_MODBUS_TIMEOUT;
			}
		}
		 break;

		/////////////////////////////////////////////////////////////////////////////////////////
		case IDOK: {
			if( NULL != lpMainData ) {
				EndDialog(hDlg, 1);
			} else {
				EndDialog(hDlg, 0);
			}
		}
		 break;

		case IDCANCEL: {
			EndDialog(hDlg, 2);
		}
		 break;
		}
	 return TRUE;
	}

    return FALSE;
}