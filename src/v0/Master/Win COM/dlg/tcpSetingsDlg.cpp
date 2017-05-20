#include "stdafx.h"

#include "../com.h"
#include "tcpSetingsDlg.h"

LRESULT CALLBACK tcpSetingsWndProc(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	LP_MAIN_DATA lpMainData = mainDataGet(GetParent(hDlg));

	switch(message){
	case WM_INITDIALOG:
		Edit_SetText(GetDlgItem(hDlg, IDC_IPADDRESS), lpMainData->mbMaster.szIP);
		Edit_SetText(GetDlgItem(hDlg, IDC_EDIT_TCP_PORT), lpMainData->mbMaster.szTcpPort);
	 return TRUE;

	case WM_COMMAND:
		
		switch(LOWORD(wParam)) {
		case IDOK: {
			if(NULL != lpMainData) {
				Edit_GetText(GetDlgItem(hDlg, IDC_IPADDRESS), lpMainData->mbMaster.szIP, 16);
				Edit_GetText(GetDlgItem(hDlg, IDC_EDIT_TCP_PORT), lpMainData->mbMaster.szTcpPort, 12);

				EndDialog(hDlg, 1);
			} else {
				EndDialog(hDlg, 0);
			}
		}
		 break;

		case IDCANCEL: EndDialog(hDlg, 2); break;
		}

	 return TRUE;
	}
    return FALSE;
}