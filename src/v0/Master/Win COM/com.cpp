/*	
	Last Update:	20.08.2011 - Add DAC Support.
	Last Update:	20.08.2011 - Add DAC Support.
	Last Update:	08.05.2012
	Full Channge:	09.01.2013 - Add ModBUS Master LIB
	Last Update:	11.01.2013
	Last Update:	26.02.2013

	Full Channge:	01.08.2015 - For IO Module, Board v.0.0.8
	Last Update:	25.08.2015
*/
#include "stdafx.h"
#include "math.h"
#include "com.h"

unsigned char inPort[32], outPort[16];
unsigned int arrADC[7], arrDAC[2] = { 0, 0 };

int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
	return DialogBox(hInstance, MAKEINTRESOURCE( IDD_MAIN ), NULL, (DLGPROC)mainWndFunc);
}

LRESULT CALLBACK mainWndFunc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	int wmId	= LOWORD(wParam);
	int wmEvent	= HIWORD(wParam);
	LP_MAIN_DATA lpMainData = mainDataGet(hwnd);

	switch(msg) {
	case WM_INITDIALOG: {
		int i;
		HWND hwndCombo;
		char szBuffer[100];

		InitCommonControls();
		LoadIcon((HINSTANCE)GetWindowLong(hwnd, GWL_HINSTANCE), (LPCTSTR)IDD_MAIN);

		lpMainData = (LP_MAIN_DATA)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(MAIN_DATA));
		if(NULL == lpMainData) {	
			EndDialog(hwnd, 0);
			return FALSE;
		}
		mainDataSet(hwnd, lpMainData);

		lpMainData->hwnd = hwnd;
		lpMainData->hInst = (HINSTANCE)GetWindowLong(hwnd, GWL_HINSTANCE);
		//////////////////////////////////////////////////////////////////////////////
		wsprintf( lpMainData->mbMaster.szIP, "192.168.0.11" );
		wsprintf( lpMainData->mbMaster.szTcpPort, "502" );
		//////////////////////////////////////////////////////////////////////////////
		lpMainData->mbMaster.hCom = INVALID_HANDLE_VALUE;
		//////////////////////////////////////////////////////////////////////////////
		lpMainData->hComThread = INVALID_HANDLE_VALUE;
		////////////////////////////////////////////////////////////////////////////////////////
		hwndCombo = GetDlgItem(hwnd, IDC_COMBO_SLAVE_ID);
		for( i = 1; i < 248; i++ ) {
			sprintf(szBuffer, "%d", i);
			ComboBox_AddString(hwndCombo, szBuffer);
		}
		lpMainData->SlaveID = 1;
		ComboBox_SetCurSel(GetDlgItem(hwnd, IDC_COMBO_SLAVE_ID), lpMainData->SlaveID - 1);

		memset(inPort, 0, sizeof(inPort));
		memset(outPort, 0, sizeof(outPort));

		Button_SetCheck(GetDlgItem(hwnd, IDC_RADIO_DIRECT_ACCESS), TRUE);
		LoadMenu((HINSTANCE)GetWindowLong(hwnd, GWL_HINSTANCE), MAKEINTRESOURCE(IDR_MENU_MAIN));

		for( i = 1; i < 9; i++ ) {
			char buffer[33];
			sprintf(buffer, "COM%d", i);
			ComboBox_AddString(GetDlgItem(hwnd, IDC_COMBO_PORT), buffer);
		}

		ComboBox_SetCurSel(GetDlgItem(hwnd, IDC_COMBO_PORT), 0);

		for( i = 0; i < 2; i++ ) {
			HWND hwndTrack = GetDlgItem(hwnd, i + IDC_SLIDER_AO0);

			SendMessage(hwndTrack, TBM_SETPOS, TRUE, 0);
			SendMessage(hwndTrack, TBM_SETTICFREQ, 10, 0);
		}

		Edit_SetText(GetDlgItem(hwnd, IDC_EDIT_DAC0_VALUE), "Set DAC0 Value");
		Edit_SetText(GetDlgItem(hwnd, IDC_EDIT_DAC1_VALUE), "Set DAC1 Value");
		///////////////////////////////////////////////////////////////////////////////
		const HWND hDesktop = GetDesktopWindow( );
		RECT desktopRect, rect;

		GetWindowRect( hDesktop, &desktopRect );
		GetWindowRect( hwnd, &rect );

		MoveWindow
		(
			hwnd,
			( desktopRect.right - (rect.right - rect.left) ) / 2,
			( desktopRect.bottom - (rect.bottom - rect.top) ) / 2,
			rect.right - rect.left,
			rect.bottom - rect.top,
			TRUE
		);
		///////////////////////////////////////////////////////////////////////////////
	}
	 return TRUE;
	
	case WM_COMMAND:
		switch(wmEvent) {
		
		case CBN_SELCHANGE: {
			HWND hwndCombo = (HWND)lParam;

			switch(wmId) {
			case IDC_COMBO_SLAVE_ID:
				lpMainData->SlaveID = ComboBox_GetCurSel(hwndCombo) + 1;
			 break;
			}
		}
		 return TRUE;

		case BN_CLICKED:
			if(wmId >= IDC_Q0 && wmId <= IDC_Q11) {

				if( !lpMainData->flagWriteSigleCoil ) {
					lpMainData->idWriteSigleCoil = wmId - IDC_Q0;
					lpMainData->flagWriteSigleCoil = 1;
				}

				return TRUE;
			}

			switch(wmId) {

			case IDC_BUTTON_ENC0_GET_OFFSET:
				if (!lpMainData->flagButtonOnClick) {
					lpMainData->flagButtonOnClick = IDC_BUTTON_ENC0_GET_OFFSET;
				}
			 break;
			
			case IDC_BUTTON_ENC0_SET_OFFSET:
				if (!lpMainData->flagButtonOnClick) {
					lpMainData->flagButtonOnClick = IDC_BUTTON_ENC0_SET_OFFSET;
				}
			 break;

			case IDM_RUN:
				lpMainData->flagRunStop = IDM_RUN;
			 break;

			case IDM_STOP:
				lpMainData->flagRunStop = IDM_STOP;
			 break;

			case IDM_CONNECTION_SETINGS:
				if( INVALID_HANDLE_VALUE != lpMainData->mbMaster.hCom ) {
					COMMCONFIG CC;
					char szBuffer[11];

					sprintf(szBuffer, "COM%d", lpMainData->mbMaster.uiComIndex);

					CC.dcb = lpMainData->mbMaster.dcb;
					CommConfigDialog(szBuffer, NULL, &CC);
					lpMainData->mbMaster.dcb.Parity = CC.dcb.Parity;
					lpMainData->mbMaster.dcb.BaudRate = CC.dcb.BaudRate;
					lpMainData->mbMaster.dcb.StopBits = CC.dcb.StopBits;
					SetCommState(lpMainData->mbMaster.hCom, &lpMainData->mbMaster.dcb);
				}
			 return TRUE;

			case IDM_CONNECTION_COMSELECT_COM1: case IDM_CONNECTION_COMSELECT_COM2:
			case IDM_CONNECTION_COMSELECT_COM3: case IDM_CONNECTION_COMSELECT_COM4:
			case IDM_CONNECTION_COMSELECT_COM5: case IDM_CONNECTION_COMSELECT_COM6:
			case IDM_CONNECTION_COMSELECT_COM7: case IDM_CONNECTION_COMSELECT_COM8: {
				HMENU hMenu = GetMenu(hwnd);
				unsigned int com = 1 + (wmId - IDM_CONNECTION_COMSELECT_COM1);

				SendMessage(hwnd, WM_COMMAND, BN_CLICKED<<16 | IDM_CONNECTION_DISCONNECT, 0);

				if( mbMasterInit(&lpMainData->mbMaster, 1) ) {
					return FALSE;
				}

				if( !mbRTUMasterConnect( &lpMainData->mbMaster, com ) ) {
					lpMainData->mbMaster.uiComIndex = com;
					////////////////////////////////////////////////////////////////////////////////////////////
					EnableMenuItem(hMenu, IDM_CONNECTION_SETINGS, MF_ENABLED);
					EnableMenuItem(hMenu, IDM_CONNECTION_DISCONNECT, MF_ENABLED);

					CheckMenuRadioItem( hMenu, IDM_CONNECTION_COMSELECT_COM1,
										IDM_CONNECTION_COMSELECT_COM8,
										wmId, MF_BYCOMMAND | MF_CHECKED
					);
					////////////////////////////////////////////////////////////////////////////////////////////
					DWORD dwThreadId;
					lpMainData->hComThread = CreateThread(NULL, 4096, comThreadFunc, lpMainData, 0, &dwThreadId);
					SetThreadPriority(lpMainData->hComThread,	THREAD_PRIORITY_TIME_CRITICAL);
					////////////////////////////////////////////////////////////////////////////////////////////
				}
			}
			 return TRUE;

			case IDM_CONNECTION_TCP_IP: {
				HMENU hMenu = GetMenu(hwnd);

				SendMessage(hwnd, WM_COMMAND, BN_CLICKED<<16 | IDM_CONNECTION_DISCONNECT, 0);

				if( mbMasterInit(&lpMainData->mbMaster, 2) ) {
					return FALSE;
				}

				switch(DialogBox(lpMainData->hInst, MAKEINTRESOURCE(IDD_TCP_SETINGS), hwnd, (DLGPROC)tcpSetingsWndProc)) {
				case 0:
				 return false;
				case 1:
				 break;
				case 2:
				 return false;

				default: return false;
				}

				if( mbTCPMasterConnect(&lpMainData->mbMaster, lpMainData->mbMaster.szIP, lpMainData->mbMaster.szTcpPort) ) {
					return FALSE;
				}
				////////////////////////////////////////////////////////////////////////////////////////////
				EnableMenuItem(hMenu, IDM_CONNECTION_DISCONNECT, MF_ENABLED);
				CheckMenuRadioItem( hMenu, IDM_CONNECTION_TCP_IP,
									IDM_CONNECTION_TCP_IP,
									wmId, MF_BYCOMMAND | MF_CHECKED
				);
				////////////////////////////////////////////////////////////////////////////////////////////
				DWORD dwThreadId;
				lpMainData->hComThread = CreateThread(NULL, 4096, comThreadFunc, lpMainData, 0, &dwThreadId);
				SetThreadPriority(lpMainData->hComThread,	THREAD_PRIORITY_TIME_CRITICAL);
				////////////////////////////////////////////////////////////////////////////////////////////
			}
			 return TRUE;

			case IDM_CONNECTION_DISCONNECT: {
				HMENU hMenu = GetMenu(hwnd);

				EnableMenuItem(hMenu, IDM_CONNECTION_SETINGS, MF_GRAYED);
				EnableMenuItem(hMenu, IDM_CONNECTION_DISCONNECT, MF_GRAYED);

				CheckMenuRadioItem( hMenu, IDM_CONNECTION_COMSELECT_COM1,
									IDM_CONNECTION_TCP_IP,
									wmId, MF_BYCOMMAND | MF_UNCHECKED
				);

				if( INVALID_HANDLE_VALUE != lpMainData->hComThread ) {
					TerminateThread(lpMainData->hComThread, 0);
					CloseHandle(lpMainData->hComThread);
					lpMainData->hComThread = INVALID_HANDLE_VALUE;
				}

				mbMasterDisconnect( &lpMainData->mbMaster );
			}
			 return TRUE;
			// --------------------------------------------------------------------------------------------------------------------
			case ID_HELP_ABOUT:
				DialogBox(lpMainData->hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hwnd, (DLGPROC)AboutWndProc);
			 break;

			case IDM_MODBUS_TIMEOUT:
				DialogBox( lpMainData->hInst, MAKEINTRESOURCE( IDD_MODBUS_TIMEOUT ), hwnd, (DLGPROC)mbTimeOutWndProc );
			 return TRUE;
			// --------------------------------------------------------------------------------------------------------------------
			case ID_ADC_CONST: DialogBox(lpMainData->hInst, MAKEINTRESOURCE(IDD_ADC_CONST), hwnd, (DLGPROC)adcConstWndProc); break;
			case ID_DAC_CONST: DialogBox(lpMainData->hInst, MAKEINTRESOURCE(IDD_DAC_CONST), hwnd, (DLGPROC)dacConstWndProc); break;

			case ID_DAC_DAC0_DIRECTACCESS:
			case ID_DAC_DAC0_000VTO1000V:
				if( !lpMainData->arrDacNewAccess[0] ) {
					lpMainData->arrDacNewAccess[0] = 1 + (wmId - ID_DAC_DAC0_DIRECTACCESS);
				}
			 return TRUE;

			case ID_DAC_DAC1_DIRECTACCESS:
			case ID_DAC_DAC1_000VTO1000V:
				if( !lpMainData->arrDacNewAccess[1] ) {
					lpMainData->arrDacNewAccess[1] = 1 + (wmId - ID_DAC_DAC1_DIRECTACCESS);
				}
			 return TRUE;

			case IDCANCEL:
				if(NULL != lpMainData) {

					if( INVALID_HANDLE_VALUE != lpMainData->hComThread ) {
						TerminateThread(lpMainData->hComThread, 0);
						CloseHandle(lpMainData->hComThread);
						lpMainData->hComThread = INVALID_HANDLE_VALUE;
					}

					mbMasterDisconnect( &lpMainData->mbMaster );
					
					HeapFree(GetProcessHeap (), GWL_USERDATA, lpMainData);
				}

				EndDialog(hwnd, 0);
			 return TRUE;
			}
		 break;
		}
	 break;

	case WM_HSCROLL: {
		int i;

		UINT wNotifyCode	= HIWORD(wParam);	// notification code 
		UINT wmId			= LOWORD(wParam);	// item, control, or accelerator identifier 
		HWND hwndCtl		= (HWND)lParam;		// handle of control 

		int nScrollCode = (int)LOWORD(wParam);			// scroll bar value 
		unsigned int nPos = (unsigned int)HIWORD(wParam);		// scroll box position 

		switch(nScrollCode) {
		case SB_THUMBTRACK:
		case SB_THUMBPOSITION:
			for(i = 0; i < 2; i++) {
				HWND hwndTrack = GetDlgItem(hwnd, i + IDC_SLIDER_AO0);

				if(hwndTrack == hwndCtl) {
					char buffer[100];
					
					arrDAC[i] = nPos;
					switch(lpMainData->DAC_Access[i]) {
					case 1: sprintf(buffer, "DAC[%d] = %u", i, 2 * nPos); break;
					case 2: sprintf(buffer, "DAC[%d] = %.2f V", i, nPos / 100.0); break;
					}

					Edit_SetText(GetDlgItem(hwnd, i + IDC_EDIT_DAC0_VALUE), buffer);
					break;
				}
			}
		 break;
		}
	}
	 break;

	case 1045: {

		switch( lParam ) {
		case FD_CONNECT:
			//MessageBeep( MB_OK );
			//SendMessage( hStatus, SB_SETTEXT, 0, (LPARAM)"Connection Established." );
		 break;

		case FD_CLOSE:
			//if( s ) {
			//	closesocket( s );
			//}

			//WSACleanup( );

			//SendMessage( hStatus, SB_SETTEXT, 0, (LPARAM)"Connection to Remote Host Lost." );
		 break;

		case FD_READ:
			char buffer[80];

			memset( buffer, 0, sizeof( buffer ) );
			//recv( s, buffer, sizeof( buffer ) - 1, 0 );
			//GetTextandAddLine( buffer, hwnd, ID_EDIT_DATA );
		 break;

		case FD_ACCEPT: {
			//SOCKET TempSock = accept( s, (struct sockaddr*)&from, &fromlen );
			//char szAcceptAddr[100];
			
			//s = TempSock;

			//MessageBeep( MB_OK );

			//wsprintf( szAcceptAddr, "Connection from [%s] accepted.", inet_ntoa(from.sin_addr) );
			//SendMessage( hStatus, SB_SETTEXT, 0, (LPARAM)szAcceptAddr );
		}
		 break;
		}

	}
	 return TRUE;

	}

	return FALSE;
}

DWORD WINAPI comThreadFunc(LPVOID lpParam)
{
	char t[100];
	int iResult;
	char szBuffer[100];
	unsigned int i = 0, counter = 0;
	LP_MAIN_DATA lpMainData = (LP_MAIN_DATA)lpParam;
	static float max = 0, min = 0;

	lpMainData->DAC_Access[0] = MODBusDacGetAccess(&lpMainData->mbMaster, lpMainData->SlaveID, 0);
	lpMainData->DAC_Access[1] = MODBusDacGetAccess(&lpMainData->mbMaster, lpMainData->SlaveID, 1);

	int time_div = 0;
	int address = 10;

	while (1) {
		unsigned int hreg[50];

		iResult = mbReadHoldingRegisters(&lpMainData->mbMaster, lpMainData->SlaveID, 40000, 30, hreg);
		if (!iResult) {
			int current_a, current_b, dc_voltage, ai0, uvw, encoder0_raw, encoder0_angle, encoder1_pos, offset;
			int rpm_t, rpm_m, rpm_tm;
			int dc_current;
			int mcu_rpm;
			float sp_speed;
			float mcu_f_rpm_t;
			int sp_pos_tim8, pos_error, temp_32;

			current_a = mbU16toSI(hreg[0]);
			current_b = mbU16toSI(hreg[1]);
			dc_voltage = mbU16toSI(hreg[2]);
			ai0 = mbU16toSI(hreg[3]);
			
			uvw = hreg[4];
			encoder0_angle = mbU16toSI(hreg[5]);
			encoder0_raw = mbU16toSI(hreg[6]);

			encoder1_pos = mbU32toSI(hreg[8] << 16 | hreg[7]);

			rpm_m = mbU16toSI(hreg[11]);
			rpm_t = hreg[12];
			dc_current = mbU16toSI(hreg[13]);

			mcu_rpm = 0; mbU32toSI(hreg[15] << 16 | hreg[14]);

			sp_pos_tim8 = mbU32toSI(hreg[17] << 16 | hreg[16]);
			pos_error = mbU32toSI(hreg[19] << 16 | hreg[18]);
			temp_32 = mbU32toSI(hreg[21] << 16 | hreg[20]);

			mcu_f_rpm_t = 0.01f * mbU32toSI(hreg[23] << 16 | hreg[22]);
			sp_speed = mbU32toSI(hreg[25] << 16 | hreg[24]) / 100.0f;

			sprintf(szBuffer, "CA: %d", current_a);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_ADC0), szBuffer);

			sprintf(szBuffer, "CB: %d", current_b);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_ADC1), szBuffer);

			sprintf(szBuffer, "Ubus: %d", dc_voltage);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_ADC2), szBuffer);

			sprintf(szBuffer, "AI0: %d", ai0 - 2047);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_ADC3), szBuffer);

			sprintf(szBuffer, "S_fb: %d", dc_current);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_ADC4), szBuffer);

			sprintf(szBuffer, "%d", 0x07 & uvw);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_HALL_POS), szBuffer);

			sprintf(szBuffer, "%d", encoder0_raw);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_ENCODER0_RAW), szBuffer);

			sprintf(szBuffer, "%d", encoder0_angle);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_ENC0_ANGLE), szBuffer);

			sprintf(szBuffer, "%d", encoder1_pos);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_ENC1_ABS_POS), szBuffer);

			sprintf(szBuffer, "%d", sp_pos_tim8);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_SP_POS_TIM8), szBuffer);

			float ftemp;

			ftemp = pos_error * ( 10.0f / 8192.0f ); // Винт, 10 мм/об.

			if (fabs(ftemp) > fabs(max)) {
				max = ftemp;
			}

			sprintf(szBuffer, "%d | %4.3f mm.", pos_error, ftemp);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_POS_ERROR), szBuffer);

			sprintf(szBuffer, "MAX: %4.3f mm.", max);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_POS_ERROR_MAX), szBuffer);

			//ftemp = (60.0f * temp_32) / 8196.0f; // Ts = 1.0 s.
			//ftemp = 60.0f * ( ( temp_32 * (1.0f/0.0025f) ) * ( 1.0f / 8192.0f ) );

			ftemp = temp_32;
			sprintf(szBuffer, "%8.2f", ftemp/100.0f);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_TEMP_32), szBuffer);

			///////////////////////////////////////////////////////////////////////////////////////
			sprintf(szBuffer, "%d", rpm_m);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_M_RAW), szBuffer);

			sprintf(szBuffer, "%d", rpm_t);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_T_RAW), szBuffer);

			sprintf(szBuffer, "%4.2f", mcu_f_rpm_t );
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_MT_RAW), szBuffer);
			///////////////////////////////////////////////////////////////////////////////////////
			const float P = 8192.0f;
			const float Ts = 0.00125f;
			const float fc = 3*4000000.0f;
			float f_rpm_m, f_rpm_t, f_rpm_mt;

			f_rpm_m = 60.0f * ((float)rpm_m / (P * Ts));
			if (rpm_m < 0) {
				rpm_t = -rpm_t;
			}

			if (rpm_t) {
				f_rpm_t = 60.0f * (fc / (P * (float)rpm_t));
			} else {
				f_rpm_t = 0.0f;
			}

			if (f_rpm_m + f_rpm_t) {
				f_rpm_mt = 2*(f_rpm_m * f_rpm_t) / (f_rpm_m + f_rpm_t);
			} else {
				f_rpm_mt = 0.0f;
			}
			///////////////////////////////////////////////////////////////////////////////////////
			sprintf(szBuffer, "%4.2f", f_rpm_m);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_M), szBuffer);
			sprintf(szBuffer, "%4.2f", f_rpm_t);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_T), szBuffer);
			sprintf(szBuffer, "%4.2f", f_rpm_mt);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_RPM_MT), szBuffer);
			///////////////////////////////////////////////////////////////////////////////////////
			sprintf(szBuffer, "%4.2f", (float)sp_speed);
			Static_SetText(GetDlgItem(lpMainData->hwnd, IDC_STATIC_SP_SPEED), szBuffer);
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}

		switch (lpMainData->flagButtonOnClick) {
		case IDC_BUTTON_ENC0_GET_OFFSET: {
			int offset;

			iResult = mbReadHoldingRegisters(&lpMainData->mbMaster, lpMainData->SlaveID, 40009, 1, (unsigned int*)&offset);
			if (!iResult) {
				sprintf(szBuffer, "%d", mbU16toSI(offset));
				Edit_SetText(GetDlgItem(lpMainData->hwnd, IDC_EDIT_ENC0_OFFSET), szBuffer);

				lpMainData->flagButtonOnClick = 0;
			}
		}
		 break;

		case IDC_BUTTON_ENC0_SET_OFFSET:
			int offset;

			Edit_GetText(GetDlgItem(lpMainData->hwnd, IDC_EDIT_ENC0_OFFSET), szBuffer, 5 );
			offset = atoi(szBuffer);

			iResult = mbWriteSingleRegister(&lpMainData->mbMaster, lpMainData->SlaveID, 40009, (unsigned int)offset);
			if (!iResult) {
				lpMainData->flagButtonOnClick = 0;
			}
		 break;
		}

		switch (lpMainData->flagRunStop) {
		case IDM_RUN:
		case IDM_STOP:
			unsigned int value = 0;

			max = 0;
			if (lpMainData->flagRunStop == IDM_RUN) {
				value = 1;
			}

			Sleep(1);
			mbWriteSingleRegister(&lpMainData->mbMaster, lpMainData->SlaveID, 15, value);
			Sleep(1);

			lpMainData->flagRunStop = 0;
			break;
		}

		Sleep(10);
	}{
		//lpMainData->SlaveID = address;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if( time_div == 1 ) {
			iResult = mdReadDiscreteInputs(&lpMainData->mbMaster, lpMainData->SlaveID, (char*)inPort, 0, 25 );
			if( !iResult ) {
				for( i = 0; i < 10; i++ ) {
					sprintf(szBuffer, "I%d: %s", i, (1 & inPort[i]) ? "ON" : "OFF");
					Static_SetText(GetDlgItem(lpMainData->hwnd, i + IDC_I0), szBuffer);
				}

				sprintf( szBuffer, "isRun: %d", (1 & inPort[23]) );
				Static_SetText( GetDlgItem(lpMainData->hwnd, IDC_IS_RUN), szBuffer );
			} else {
//				break;
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if( time_div == 2 || lpMainData->flagWriteSigleCoil ) {
			iResult = mbReadCoils(&lpMainData->mbMaster, lpMainData->SlaveID, (char*)outPort, 0, 12);
			if( !iResult ) {
				for( i = 0; i < 12; i++ ) {
					sprintf(szBuffer, "Q%d (%d)", i, outPort[i]);
					Button_SetText( GetDlgItem( lpMainData->hwnd, i + IDC_Q0), szBuffer );
				}

				if( lpMainData->flagWriteSigleCoil ) {
					unsigned char temp[12];

					memcpy( temp, outPort, sizeof(temp) );
					temp[ lpMainData->idWriteSigleCoil ] ^= 1;

					iResult = mbWriteMultipleCoils( &lpMainData->mbMaster, lpMainData->SlaveID, 0, 12, temp );
					if( !iResult ) {
					} else {
//						break;
					}

					lpMainData->flagWriteSigleCoil = 0;
				}
			} else {
//				break;
			}
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ADC:
		switch( lpMainData->flagAdcConst ) {

		case IDC_BUTTON_ADC0_SET_CONST:
			lpMainData->flagAdcConst = 0;

			Edit_GetText( GetDlgItem(lpMainData->hwndAdcDialog, IDC_EDIT_ADC0_CONST + lpMainData->flagAdcConstId), szBuffer, 10 );

			iResult = MODBusSetAdcConst( &lpMainData->mbMaster, lpMainData->SlaveID, lpMainData->flagAdcConstId, atoi(szBuffer) );
			if( !iResult ) {
			} else {
//				break;
			}
		 break;

		case IDC_BUTTON_ADC_READ_CONST:
			lpMainData->flagAdcConst = 0;

			iResult = MODBusReadADCConst( &lpMainData->mbMaster, lpMainData->SlaveID, (unsigned int*)&lpMainData->arrAdcConst );
			if( !iResult ) {
				for( i = 0; i < 7; i++ ) {
					sprintf(szBuffer, "%d", lpMainData->arrAdcConst[i]);
					Edit_SetText( GetDlgItem(lpMainData->hwndAdcDialog, IDC_EDIT_ADC0_CONST + i), szBuffer );
				}
			} else {
//				break;
			}
		 break;

		case IDC_BUTTON_ADC_LOAD_DEF_CONST:
			lpMainData->flagAdcConst = 0;

			iResult = MODBusAdcLoadDefConst( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult ) {
				SendMessage( lpMainData->hwndAdcDialog, WM_COMMAND, IDC_BUTTON_ADC_READ_CONST, 0 );
			} else {
//				break;
			}
		 break;

		case IDC_BUTTON_ADC_LOAD_FROM_EEPROM:
			lpMainData->flagAdcConst = 0;

			iResult = MODBusAdcLoadConstFromEEPROM( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult ) {
				SendMessage( lpMainData->hwndAdcDialog, WM_COMMAND, IDC_BUTTON_ADC_READ_CONST, 0 );
			} else {
//				break;
			}
		 break;

		case ID_ADC_CONST_SAVE_TO_EEPROM:
			lpMainData->flagAdcConst = 0;

			iResult = MODBusAdcSaveConst2EEPROM( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult ) {
			} else {
//				break;
			}
		 break;
		}

		if( time_div == 3 && (
			TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO_DIRECT_ACCESS)) ||
			TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO10_00V)) ||
			TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO_10_000V)) )
		) {
			iResult = MODBusReadRawADC( &lpMainData->mbMaster, lpMainData->SlaveID, arrADC );
			if( !iResult ) {
				if(TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO_DIRECT_ACCESS ))) {
					for(i = 0; i < 7; i++) {
						sprintf(t, " AI%d = %d", i, ( 0x0fff & arrADC[i] ) );
						Static_SetText( GetDlgItem(lpMainData->hwnd, i + IDC_ADC0), t );
					}
				} else {
					unsigned int adcConst[7];

					iResult = MODBusReadADCConst( &lpMainData->mbMaster, lpMainData->SlaveID, adcConst );
					if( !iResult ) {
						for( i = 0; i < 7; i++ ) {
							float f = (float)100.00 / (float)adcConst[i];

							f *= ( 0xffff & arrADC[i] );

							if( TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO10_00V)) ) {
								sprintf( (char*)t, " AI%d = %.2f V", i, f );
							} else
							if( TRUE == Button_GetCheck(GetDlgItem(lpMainData->hwnd, IDC_RADIO_10_000V)) ) {
								sprintf( (char*)t, " AI%d = %.3fV", i, f );								
							}
							Static_SetText(GetDlgItem(lpMainData->hwnd, i + IDC_ADC0), t);
						}
					} else {
//						break;
					}
				}

			} else {
//				break;
			}
		} else {
			if( TRUE == Button_GetCheck( GetDlgItem(lpMainData->hwnd, IDC_RADIO_1000) ) ) {
				iResult = MODBusReadADCValue1000( &lpMainData->mbMaster, lpMainData->SlaveID, arrADC );
				if( !iResult ) {
					for(i = 0; i < 7; i++) {
						sprintf( szBuffer, " AI%d = %d", i, (0xfff & arrADC[i]) );
						Static_SetText( GetDlgItem(lpMainData->hwnd, i + IDC_ADC0), szBuffer );
					}
				} else {
//					break;
				}
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// DAC1 and DAC0:
		if( time_div == 4 ) {
			for( i = 0; i < 2; i++ ) {

				if( lpMainData->arrDacNewAccess[i] ) {
					iResult = MODBusDacSetAccess( &lpMainData->mbMaster, lpMainData->SlaveID, i, lpMainData->arrDacNewAccess[i] );
					if( !iResult) {
					} else {
//						break;
					}
					lpMainData->arrDacNewAccess[i] = 0;
				}

				lpMainData->DAC_Access[i] = MODBusDacGetAccess( &lpMainData->mbMaster, lpMainData->SlaveID, i );
				if( !iResult) {
					switch (lpMainData->DAC_Access[i]) {
					case 1: {
						iResult = MODBusWriteRawDAC( &lpMainData->mbMaster, lpMainData->SlaveID, i, 2 * arrDAC[i] );
						if (!iResult) {
							CheckMenuRadioItem(
								GetMenu(lpMainData->hwnd),
								2 * i + ID_DAC_DAC0_DIRECTACCESS,
								2 * i + ID_DAC_DAC0_000VTO1000V,
								2 * i + ID_DAC_DAC0_DIRECTACCESS,
								MF_BYCOMMAND | MF_CHECKED
							);

							SendMessage(
								GetDlgItem(lpMainData->hwnd, IDC_SLIDER_AO0 + i),
								TBM_SETRANGE,
								(WPARAM)TRUE,				// redraw flag
								(LPARAM)MAKELONG(0, 32767)	// min. & max. positions
							);
						} else {
							//break;
						}
					}
					 break;

					case 2: {
						iResult = MODBusWriteDAC_1000( &lpMainData->mbMaster, lpMainData->SlaveID, i, arrDAC[i] );
						if (!iResult) {
							CheckMenuRadioItem(
								GetMenu(lpMainData->hwnd),
								2 * i + ID_DAC_DAC0_DIRECTACCESS,
								2 * i + ID_DAC_DAC0_000VTO1000V,
								2 * i + ID_DAC_DAC0_000VTO1000V,
								MF_BYCOMMAND | MF_CHECKED
							);

							SendMessage(
								GetDlgItem(lpMainData->hwnd, IDC_SLIDER_AO0 + i),
								TBM_SETRANGE,
								(WPARAM)TRUE,				// redraw flag
								(LPARAM)MAKELONG(0, 1000)	// min. & max. positions
							);
						} else {
							//break;
						}
					 }
					 break;
					}
				} else {
//					break;
				}
			}
		}

		switch( lpMainData->flagDacConst ) {
		case ID_DAC0_SET_CONST:
			lpMainData->flagDacConst = 0;
			
			Edit_GetText( GetDlgItem(lpMainData->hwndDacDialog, IDC_EDIT_DAC0), szBuffer, 10 );
			
			iResult = MODBusSetDacConst( &lpMainData->mbMaster, lpMainData->SlaveID, 0, atoi(szBuffer) );
			if( !iResult) {
			} else {
//				break;
			}
		 break;

		case ID_DAC1_SET_CONST:
			lpMainData->flagDacConst = 0;
			
			Edit_GetText( GetDlgItem(lpMainData->hwndDacDialog, IDC_EDIT_DAC1), szBuffer, 10 );
			
			iResult = MODBusSetDacConst( &lpMainData->mbMaster, lpMainData->SlaveID, 1, atoi(szBuffer) );
			if( !iResult) {
			} else {
//				break;
			}
		 break;

		case IDC_BUTTON_DAC_READ_CONST: {
			unsigned int value;

			lpMainData->flagDacConst = 0;

			iResult = MODBusDacReadConst( &lpMainData->mbMaster, lpMainData->SlaveID, 0, &value );
			if( !iResult) {
				sprintf(szBuffer, "%d", value);
				Edit_SetText( GetDlgItem(lpMainData->hwndDacDialog, IDC_EDIT_DAC0), szBuffer );
			} else {
//				break;
			}

			iResult = MODBusDacReadConst( &lpMainData->mbMaster, lpMainData->SlaveID, 1, &value );
			if( !iResult) {
				sprintf(szBuffer, "%d", value);
				Edit_SetText( GetDlgItem(lpMainData->hwndDacDialog, IDC_EDIT_DAC1), szBuffer );
			} else {
//				break;
			}
		}
		 break;

		case IDC_BUTTON_DAC_LOAD_DEF_CONST:
			lpMainData->flagDacConst = 0;

			iResult = MODBusDacLoadDefConst( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult) {
				SendMessage( lpMainData->hwndDacDialog, WM_COMMAND, IDC_BUTTON_DAC_READ_CONST, 0 );
			} else {
//				break;
			}
		 break;

		case IDC_BUTTON_DAC_LOAD_FROM_EEPROM:
			lpMainData->flagDacConst = 0;
			
			iResult = MODBusDacLoadConstFromEEPROM( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult) {
				SendMessage( lpMainData->hwndDacDialog, WM_COMMAND, IDC_BUTTON_DAC_READ_CONST, 0 );
			} else {
//				break;
			}
		 break;

		case ID_DAC_CONST_SAVE_TO_EEPROM:
			lpMainData->flagDacConst = 0;

			iResult = MODBusDacSaveConst2EEPROM( &lpMainData->mbMaster, lpMainData->SlaveID );
			if( !iResult) {
			} else {
//				break;
			}
		 break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		switch( lpMainData->uiFlagButton ) {
		case IDC_BUTTON_WRITE_MODBUS_TIMEOUT: {
			
			mbWriteSingleRegister
			(
				&lpMainData->mbMaster, lpMainData->SlaveID,
				18, mbSItoU16( lpMainData->uiMbTimeOut )
			);

			lpMainData->uiFlagButton = IDC_BUTTON_READ_MODBUS_TIMEOUT;
		}
		 break;

		case IDC_BUTTON_READ_MODBUS_TIMEOUT: {
			iResult = mbReadHoldingRegisters
			(
				&lpMainData->mbMaster, lpMainData->SlaveID,
				18, 1, &lpMainData->uiMbTimeOut
			);
			if( !iResult ) {
				sprintf( szBuffer, "%d", lpMainData->uiMbTimeOut );
				Edit_SetText( GetDlgItem( lpMainData->hwndMbTimeOutDialog, IDC_EDIT_MODBUS_TIMEOUT ), szBuffer );
			}
			lpMainData->uiFlagButton = 0;
		}
		 break;
		}

		if( ++time_div > 5 ) {
			time_div = 0;
		}

		Sleep( 1 );
	}

	mbMasterDisconnect(&lpMainData->mbMaster);

	if( 1 == lpMainData->mbMaster.ascii_or_rtu_or_tcp ) {
		TerminateThread(lpMainData->hComThread, 0);
		CloseHandle(lpMainData->hComThread);
		lpMainData->hComThread = INVALID_HANDLE_VALUE;
	}

	return false;
}
///////////////////////////////////////////////////////////////////////////////////
int MODBusReadADCValue1000
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcValue
)
{
	return mbReadInputRegister(lpMb, deviceID, 7, 7, adcValue);
}

int MODBusReadRawADC
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcValue
)
{
	return mbReadInputRegister(lpMb, deviceID, 0, 7, adcValue);
}

int MODBusReadADCConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcConst
)
{
	return mbReadHoldingRegisters( lpMb, deviceID, 5, 7, adcConst );
}

int MODBusDacReadConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac, unsigned int *value
)
{
	return mbReadHoldingRegisters( lpMb, deviceID, 12 + dac, 1, value );
}

int MODBusDacSetAccess
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int access
)
{
	if( dac > 1 || access < 1 || access > 2 ) {
		return 1;
	}

	if(1 == dac) {
		access <<= 2;
	}

	access <<= 8;

	return mbWriteSingleRegister(lpMb , deviceID, 14, 0x1000 | access);
}

int MODBusDacGetAccess( LPMB lpMb, unsigned char deviceID, unsigned char dac )
{
	unsigned int access = 0, regValue;
	
	if( dac < 2 ) {
		if( !mbReadHoldingRegisters( lpMb, deviceID, 14, 1, &regValue) ) {
			regValue >>= 8;

			if(!dac) {
				access = 3 & regValue;
			} else {
				access = 3 & (regValue>>2);
			}
		}
	}

	return access;
}

int MODBusSetAdcConst
( 
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char adc, unsigned int new_const
)
{
	return mbWriteSingleRegister( lpMb, deviceID, 5 + adc, new_const );	
}

int MODBusAdcLoadDefConst( LPMB lpMb, unsigned char deviceID )
{
	return mbWriteSingleRegister( lpMb, deviceID, 14, 32 );	
}

int MODBusAdcLoadConstFromEEPROM( LPMB lpMb, unsigned char deviceID )
{	
	return mbWriteSingleRegister( lpMb, deviceID, 14, 64 );	
}

int MODBusAdcSaveConst2EEPROM( LPMB lpMb, unsigned char deviceID )
{	
	return mbWriteSingleRegister( lpMb, deviceID, 14, 128 );	
}

int MODBusWriteRawDAC
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int value
)
{
	if( dac > 1 ) {
		return 1;
	}

	return mbWriteSingleRegister( lpMb, deviceID, dac, value);
}

int MODBusWriteDAC_1000
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int value
)
{
	if( dac > 1 ) {
		return 1;
	}

	return mbWriteSingleRegister(lpMb, deviceID, 2 + dac, value);
}

int MODBusSetDacConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int new_const
)
{
	return mbWriteSingleRegister(lpMb, deviceID, 12 + dac, new_const);
}

int MODBusDacLoadDefConst( LPMB lpMb, unsigned char deviceID )
{
	return mbWriteSingleRegister(lpMb, deviceID, 14, 32<<8);
}

int MODBusDacLoadConstFromEEPROM( LPMB lpMb, unsigned char deviceID )
{
	return mbWriteSingleRegister(lpMb, deviceID, 14, 64<<8);
}

int MODBusDacSaveConst2EEPROM( LPMB lpMb, unsigned char deviceID )
{
	return mbWriteSingleRegister(lpMb, deviceID, 14, 128<<8);
}
///////////////////////////////////////////////////////////////////////////////////

LP_MAIN_DATA mainDataGet(HWND hwnd)
{
	return (LP_MAIN_DATA)GetWindowLong(hwnd, GWL_USERDATA);
}

void mainDataSet(HWND hwnd, LP_MAIN_DATA lpMainData)
{
	SetWindowLong(hwnd, GWL_USERDATA, (LONG)lpMainData);
}
