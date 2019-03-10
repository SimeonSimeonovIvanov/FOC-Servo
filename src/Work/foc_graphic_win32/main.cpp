// main.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "resource.h"
#include "math.h"
#include "foc.h"

#include "stdio.h"

#define MAX_LOADSTRING 100

#define PI 3.14159265f

#define deg_to_rad( deg )	( deg * ( PI / 180.f ) )

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];								// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];								// The title bar text

// Foward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_MAIN, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow)) 
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, (LPCTSTR)IDC_MAIN);

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0)) 
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg)) 
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage is only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX); 

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= (WNDPROC)WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, (LPCTSTR)IDI_MAIN);
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= (LPCSTR)IDC_MAIN;
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, (LPCTSTR)IDI_SMALL);

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HANDLE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	HWND hWnd;

	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow
	(
		szWindowClass, szTitle, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL
	);
	
	if( !hWnd ) {
		return FALSE;
	}

	ShowWindow( hWnd, nCmdShow );
	UpdateWindow( hWnd );

	return TRUE;
}

//
//  FUNCTION: WndProc(HWND, unsigned, WORD, LONG)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	static int arrGraphic[10][2000];
	static int steep = 3;
	static int gr_size_x = 400 * steep;

	static MC_FOC stFoc;

	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch( message ) {		
	case WM_CREATE: {
		int counter = 0;
		int i, angle = 0;
		float scale = 2.5f;
		
		svpwmInitSin3Table();
		mcFocInitStruct(&stFoc);

		float Ia, Ib, Ic;

		for( i = 0; i < gr_size_x; i++ ) {
			angle = ( i / steep );

			Ia = fSinAngle(angle);
			Ib = fSinAngle(angle - 120);
			///////////////////////////////////////////////////////////////////
			Ia = Ia * 20.0f;
			Ib = Ib * 20.0f;
			Ic = -Ia -Ib;
			///////////////////////////////////////////////////////////////////
			mcFocSetAngle(&stFoc, angle);
			mcFocSetCurrent(&stFoc, Ia, Ib);
			///////////////////////////////////////////////////////////////////
			mcClark(&stFoc);
			mcPark(&stFoc);
			///////////////////////////////////////////////////////////////////
			/*
			stFoc.Vd = pid( Zero_Sp, stFoc.Id );
			stFoc.Vq = pid(   Iq_Sp, stFoc.Iq );
			*/
			stFoc.Vd = 0.0f;
			stFoc.Vq = 0.90f;
			///////////////////////////////////////////////////////////////////
			mcUsrefLimit(&stFoc);
			///////////////////////////////////////////////////////////////////
			mcInvPark(&stFoc);
			mcInvClark(&stFoc);
			///////////////////////////////////////////////////////////////////
			//mcFocSVPWM_ST2(&stFoc);
			//mcFocSVPWM_ST1(&stFoc);
			//mcFocSVPWM0(&stFoc);
			//mcFocSPWM(&stFoc);
			
			//mcFocSVPWM_TTHI(&stFoc);
			mcFocSVPWM_STHI(&stFoc);
			///////////////////////////////////////////////////////////////////
			arrGraphic[0][i] = Ia;
			arrGraphic[1][i] = Ib;
			arrGraphic[2][i] = Ic;
			arrGraphic[0][i] = stFoc.X;
			arrGraphic[1][i] = stFoc.Y;
			arrGraphic[2][i] = stFoc.Z;
			//arrGraphic[0][i] = stFoc.Va * 20;
			//arrGraphic[1][i] = stFoc.Vb *20;
			//arrGraphic[2][i] = stFoc.Vc *20;
			
			// ----------------------------------------------------------------
			if( counter++ <= 30 * steep ) {
				arrGraphic[10][i] = +1;
			} else {
				arrGraphic[10][i] = -1;
			}
			if( counter > 60 * steep ) {
				counter = 0;
			}
			// ----------------------------------------------------------------

			arrGraphic[3][i] = stFoc.Valpha*50;
			arrGraphic[4][i] = stFoc.Vbeta*50;

			//arrGraphic[5][i] = stFoc.Id;
			//arrGraphic[6][i] = stFoc.Iq;
			arrGraphic[5][i] = stFoc.PWM1;
			arrGraphic[6][i] = stFoc.PWM2;
			arrGraphic[7][i] = stFoc.PWM3;

			//arrGraphic[8][i] = sqrtf(stFoc.Id*stFoc.Id + stFoc.Iq*stFoc.Iq);

			arrGraphic[0][i] *= scale;
			arrGraphic[1][i] *= scale;
			arrGraphic[2][i] *= scale;
			arrGraphic[3][i] *= scale;
			arrGraphic[4][i] *= scale;
			arrGraphic[5][i] *= scale;
			arrGraphic[6][i] *= scale;
			arrGraphic[7][i] *= scale;
			arrGraphic[8][i] *= scale;
			arrGraphic[10][i] *= scale;
		}
	}
	 break;

	case WM_COMMAND:
		wmId    = LOWORD(wParam); 
		wmEvent = HIWORD(wParam); 
		// Parse the menu selections:
		switch (wmId) {
		case IDM_ABOUT:
			DialogBox(hInst, (LPCTSTR)IDD_ABOUTBOX, hWnd, (DLGPROC)About);
		 break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
		 break;
		default: return DefWindowProc(hWnd, message, wParam, lParam);
		}
	 break;
	
	case WM_PAINT: {
		RECT rt;

		int x_size, y_size;
		int xdiv2, ydiv2;
		int i, gr_left;

		GetClientRect(hWnd, &rt);

		x_size = rt.right - rt.left;
		y_size = rt.bottom - rt.top;

		xdiv2 = x_size / 2;
		ydiv2 = y_size / 2;

		gr_left = (xdiv2 - gr_size_x / 2);

		hdc = BeginPaint(hWnd, &ps);

		for (i = 0; i < gr_size_x; i++) {
			int x_pos = gr_left + i;

			SetPixel(hdc, x_pos, ydiv2, RGB(127, 127, 127));

			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[0][i] - 300, RGB(0, 0, 255));
			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[1][i] - 300, RGB(0, 255, 0));
			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[2][i] - 300, RGB(255, 0, 0));

			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[3][i] - 200, RGB(0, 0, 255));
			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[4][i] - 100, RGB(0, 0, 255));

			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[10][i] + 0, RGB(127, 127, 127));

			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[5][i] + 200, RGB(0, 0, 255));
			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[6][i] + 200, RGB(0, 255, 0));
			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[7][i] + 200, RGB(255, 0, 0));

			SetPixel(hdc, x_pos, ydiv2 - arrGraphic[8][i] + 300, RGB(255, 0, 255));

			SetPixel(hdc, x_pos, ydiv2 - 300, RGB(255, 0, 0));
			SetPixel(hdc, x_pos, ydiv2 - 200, RGB(0, 255, 0));
			SetPixel(hdc, x_pos, ydiv2 - 100, RGB(255, 0, 255));
			//SetPixel( hdc, x_pos, ydiv2 -   0, RGB( 255, 255, 255 ) );
			SetPixel(hdc, x_pos, ydiv2 + 100, RGB(127, 127, 127));
			SetPixel(hdc, x_pos, ydiv2 + 200, RGB(127, 127, 127));
			SetPixel(hdc, x_pos, ydiv2 + 300, RGB(127, 127, 127));
		}

		int x = gr_left - 30 * (gr_size_x / 400);
		char string[100];

		for (i = 0; i < 7; i++) {	
			sprintf(string, "%d", stFoc.arrSector[i]);
			TextOut(hdc, x += 60 * (gr_size_x / 400), 360, string, strlen(string));
		}

		EndPaint(hWnd, &ps);
	}
	 break;

	case WM_DESTROY:
		PostQuitMessage(0);
	 break;

	default: return DefWindowProc(hWnd, message, wParam, lParam);
   }

   return 0;
}

// Mesage handler for about box.
LRESULT CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message) {
	case WM_INITDIALOG:
	 return TRUE;

	case WM_COMMAND:
		if(LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL) {
			EndDialog(hDlg, LOWORD(wParam));
			return TRUE;
		}
	 break;
	}

	return FALSE;
}
