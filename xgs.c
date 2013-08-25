#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <Carbon/Carbon.h>
#else
#include <stdlib.h>
#include <GL/gl.h>
#endif


static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon);
                

#define STATE_LAND 1
#define STATE_AIR 2

#define WINDOW_WIDTH 130
#define WINDOW_HEIGHT 80

#define MAX_TOKEN_BUF 100

static XPLMWindowID gWindow = NULL;
static XPLMDataRef vertSpeedRef, gearKoofRef, flightTimeRef, descrRef;
static XPLMDataRef longRef, latRef, craftNumRef;
static XPLMDataRef pitchRef, speedRef, iasRef;
static XPLMDataRef windSpeedRef, windDirRef, headingRef, declinationRef;
static char landMsg1[100],  landMsg2[100],  landMsg3[100], landMsg4[100];
static int lastState;
static float landingSpeed = 0.0f;
static float lastVSpeed = 0.0f;
static float remainingShowTime = 0.0f;

static struct {
    float pitch, speed, ias;
    float windSpeed, windDir, heading;
} landingData;

static int winPosX = 20;
static int winPosY = 600;
static int lastMouseX, lastMouseY;
static int windowCloseRequest = 0;
static XPLMMenuID xgsMenu = NULL;
static int enableLogItem;
static int logEnabled = 0;

static int logThisLanding = 0;
static char logAirportId[50];
static char logAirportName[300];
static char logAircraftNum[50];
static time_t landingTime;



#ifdef __APPLE__
int MacToUnixPath(const char * inPath, char * outPath, int outPathMaxLen)
{
    CFStringRef inStr = CFStringCreateWithCString(kCFAllocatorDefault, inPath, kCFStringEncodingMacRoman);
    if (inStr == NULL) return -1;
    CFURLRef url = CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLHFSPathStyle,0);
    CFStringRef outStr = CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle);
    if (!CFStringGetCString(outStr, outPath, outPathMaxLen, kCFURLPOSIXPathStyle)) return -1;
    CFRelease(outStr);
    CFRelease(url);
    CFRelease(inStr);
    return 0;
}
#endif


static FILE* getConfigFile(char *mode)
{
    char path[512];
    
    XPLMGetPrefsPath(path);
    XPLMExtractFileAndPath(path);
    strcat(path, XPLMGetDirectorySeparator());
    strcat(path, "xgs.prf");

#ifdef __APPLE__
    char unixPath[512];
    MacToUnixPath(path, unixPath, 512);
    
    return fopen(unixPath, mode);
#else
    return fopen(path, mode);
#endif
}


static int skipSpaces(FILE *f)
{
    int ch;
    
    do {
        ch = fgetc(f);
        if (EOF == ch)
            return -1;
    } while (isspace(ch));
    ungetc(ch, f);
    
    return 0;
}


static int getNextToken(FILE *f, char *buf)
{
    int pos = 0;
    int ch;
    
    if (skipSpaces(f))
        return -1;
    
    ch = fgetc(f);
    while ((! isspace(ch)) && (EOF != ch) && (MAX_TOKEN_BUF - 1 > pos)) {
        buf[pos++] = ch;
        ch = fgetc(f);
    }
    if (MAX_TOKEN_BUF - 1 == pos)
        return -1;
    
    buf[pos] = 0;
    return 0;
}


static int tokenToInt(char *token, int *val)
{
    char *endptr;
    int v = strtol(token, &endptr, 10);
    if (endptr && (! *endptr)) {
        *val = v;
        return 0;
    } else
        return -1;
}

static void saveConfig()
{
    FILE *f;
    
    f = getConfigFile("w");
    if (! f)
        return;
    
    fprintf(f, "%i %i %i", winPosX, winPosY, logEnabled);
    
    fclose(f);
}


#define ABORT_CONF { fclose(f); return; }

static void loadConfig()
{
    FILE *f;
    char s[MAX_TOKEN_BUF];
    
    f = getConfigFile("r");
    if (! f)
        return;
    
    if (getNextToken(f, s)) ABORT_CONF
    tokenToInt(s, &winPosX);
    if (getNextToken(f, s)) ABORT_CONF
    tokenToInt(s, &winPosY);
    if (getNextToken(f, s)) ABORT_CONF
    tokenToInt(s, &logEnabled);
    
    fclose(f);
}

#undef ABORT_CONF


static void updateLogItemState()
{
    XPLMCheckMenuItem(xgsMenu, enableLogItem, 
        logEnabled ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}


static void xgsMenuCallback(void *menuRef, void *param)
{
    logEnabled = ! logEnabled;
    updateLogItemState();
}


PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    XPLMMenuID pluginsMenu;
    int subMenuItem;
    
    loadConfig();
    
    strcpy(outName, "Landing Speed");
    strcpy(outSig, "babichev.landspeed");
    strcpy(outDesc, "A plugin that shows vertical landing speed.");

    vertSpeedRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind");
    gearKoofRef = XPLMFindDataRef("sim/flightmodel/forces/faxil_gear");
    flightTimeRef = XPLMFindDataRef("sim/time/total_flight_time_sec");
    descrRef = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
    latRef = XPLMFindDataRef("sim/flightmodel/position/latitude");
    longRef = XPLMFindDataRef("sim/flightmodel/position/longitude");
    craftNumRef = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
    
    pitchRef = XPLMFindDataRef("sim/flightmodel/position/theta");
    speedRef = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    iasRef = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
    
    windDirRef = XPLMFindDataRef("sim/weather/wind_direction_degt");
    windSpeedRef = XPLMFindDataRef("sim/weather/wind_speed_kt");
    headingRef = XPLMFindDataRef("sim/flightmodel/position/magpsi");
    declinationRef = XPLMFindDataRef("sim/flightmodel/position/magnetic_variation");
    
    landMsg1[0] = landMsg2[0] = landMsg3[0] = 0;
    XPLMRegisterFlightLoopCallback(gameLoopCallback, 0.05f, NULL);
    
    pluginsMenu = XPLMFindPluginsMenu();
    subMenuItem = XPLMAppendMenuItem(pluginsMenu, "Landing Speed", NULL, 1);
    xgsMenu = XPLMCreateMenu("Landing Speed", pluginsMenu, subMenuItem, 
                xgsMenuCallback, NULL);     
    enableLogItem = XPLMAppendMenuItem(xgsMenu, "Enable Log", NULL, 1);
    updateLogItemState();
    
    return 1;
}


static void trim(char *str)
{
    int len = strlen(str);
    len--;
    while (0 < len) {
        if (('\r' == str[len]) || ('\n' == str[len])) {
            str[len] = 0;
            len--;
        } else
            return;
    }
}


static void writeLandingToLog()
{
    FILE *f;
    char buf[512];
    
    logThisLanding = 0;
    XPLMGetSystemPath(buf);
    strcat(buf, "landing.log");

#ifdef __APPLE__
    char unixPath[512];
    MacToUnixPath(buf, unixPath, 512);

    f = fopen(unixPath, "a");
#else
    f = fopen(buf, "a");
#endif
    if (! f) return;

    strcpy(buf, ctime(&landingTime));
    trim(buf);
    fprintf(f, "%s %s %s '%s' %.3f m/s %i fpm %s\n", buf, logAircraftNum,
                logAirportId, logAirportName, fabs(landingSpeed), 
                abs(rint(landingSpeed * 60.0f * 3.2808f)), landMsg1);

    fclose(f);
}


static void closeEventWindow()
{
    if (logThisLanding)
        writeLandingToLog();
    
    if (gWindow) {
        XPLMDestroyWindow(gWindow);
        gWindow = NULL;
    }
    
    landingSpeed = 0.0f;
    landMsg1[0] = landMsg2[0] = landMsg3[0] = 0;
    remainingShowTime = 0.0f;
}

PLUGIN_API void	XPluginStop(void)
{
    closeEventWindow();
    saveConfig();
}


PLUGIN_API void XPluginDisable(void)
{
}


PLUGIN_API int XPluginEnable(void)
{
	return 1;
}


PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,
                long inMessage,	void *inParam)
{
}

void MyDrawWindowCallback(XPLMWindowID inWindowID, void *inRefcon)
{
    if (0.0f < remainingShowTime) {
        int left, top, right, bottom;
        float color[] = { 1.0, 1.0, 1.0 }; 	/* RGB White */
            
        XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
        XPLMDrawTranslucentDarkBox(left, top, right, bottom);
        XPLMDrawString(color, left + 5, top - 20, landMsg1, NULL, xplmFont_Basic);
        XPLMDrawString(color, left + 5, top - 35, landMsg2, NULL, xplmFont_Basic);
        XPLMDrawString(color, left + 5, top - 50, landMsg3, NULL, xplmFont_Basic);
        XPLMDrawString(color, left + 5, top - 65, landMsg4, NULL, xplmFont_Basic);
        
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.7, 0.7, 0.7);
        glBegin(GL_LINES);
          glVertex2i(right - 1, top - 1);
          glVertex2i(right - 7, top - 7);
          glVertex2i(right - 7, top - 1);
          glVertex2i(right - 1, top - 7);
        glEnd();
        glEnable(GL_TEXTURE_2D);
    }
}                                   

static int mouseCallback(XPLMWindowID inWindowID, int x, int y,    
                   XPLMMouseStatus inMouse, void *inRefcon)
{
    if (windowCloseRequest)
        return 1;
    
    switch (inMouse) {
        case xplm_MouseDown:
            if ((x >= winPosX + WINDOW_WIDTH - 8) && (x <= winPosX + WINDOW_WIDTH) && 
                        (y <= winPosY) && (y >= winPosY - 8))
                windowCloseRequest = 1;
            else {
                lastMouseX = x;
                lastMouseY = y;
            }
            break;
            
        case xplm_MouseDrag:
            winPosX += x - lastMouseX;
            winPosY += y - lastMouseY;
            XPLMSetWindowGeometry(gWindow, winPosX, winPosY, 
                    winPosX + WINDOW_WIDTH, winPosY - WINDOW_HEIGHT);
            lastMouseX = x;
            lastMouseY = y;
            break;
            
        case xplm_MouseUp:
            break;
    }

    return 1;
}

static void keyboardCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,    
                   char inVirtualKey, void *inRefcon, int losingFocus)
{
}

static int getCurrentState()
{
    return 0.0f != XPLMGetDataf(gearKoofRef) ? STATE_LAND : STATE_AIR;
}


static void printLandingMessage(float vy)
{
    if (vy < 0.5)
        sprintf(landMsg1, "excelent landing");
    else if (vy < 1.0)
        sprintf(landMsg1, "good landing");
    else if (vy < 1.5)
        sprintf(landMsg1, "acceptable landing");
    else if (vy < 2.0)
        sprintf(landMsg1, "hard landing");
    else if (vy < 2.5)
        sprintf(landMsg1, "you are fired!!!");
    else if (vy < 3.0)
        sprintf(landMsg1, "anybody survived?");
    else
        sprintf(landMsg1, "R.I.P.");
}


static void prepareToLog()
{
    int num;
    float lat = XPLMGetDataf(latRef);
    float lon = XPLMGetDataf(longRef);
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    
    if (XPLM_NAV_NOT_FOUND != ref)
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, logAirportId, 
                logAirportName, NULL);
    else {
        logAirportId[0] = 0;
        logAirportName[0] = 0;
    }
    landingTime = time(NULL);
    num = XPLMGetDatab(craftNumRef, logAircraftNum, 0, 49);
    logAircraftNum[num] = 0;
    logThisLanding = 1;
}


static void createEventWindow()
{
    printLandingMessage(-landingSpeed);
    sprintf(landMsg2, "Vy: %i fpm %0.1f˚", (int)abs(rint(landingSpeed * 60.0f * 3.2808f)), landingData.pitch);
    //sprintf(landMsg3, "Vy: %.3f m/s", fabs(landingSpeed));
    sprintf(landMsg3, "Vx: %i kias %i k", (int)rint(landingData.ias), (int)rint(landingData.speed / 0.5144f));
    sprintf(landMsg4, "Wind: %i k @ %i ˚", (int)rint(landingData.windSpeed), (int)rint(landingData.windDir));
    if (! gWindow)
        gWindow = XPLMCreateWindow(winPosX, winPosY, 
                    winPosX + WINDOW_WIDTH, winPosY - WINDOW_HEIGHT, 
                    1, MyDrawWindowCallback, keyboardCallback, 
                    mouseCallback, NULL);
    if (logEnabled && (! logThisLanding))
        prepareToLog();
}


static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon)
{
    int state = getCurrentState();
    float timeFromStart = XPLMGetDataf(flightTimeRef);

    if (3.0 < timeFromStart) {
        if (windowCloseRequest) {
            windowCloseRequest = 0;
            closeEventWindow();
        } else
            if (0.0f < remainingShowTime) {
                remainingShowTime -= inElapsedSinceLastCall;
                if (0.0f >= remainingShowTime)
                    closeEventWindow();
            }
    
        if ((STATE_AIR == lastState) && (STATE_LAND == state)) {
            if (lastVSpeed < landingSpeed) {
                landingSpeed = lastVSpeed;
                
                landingData.pitch = XPLMGetDataf(pitchRef);
                landingData.speed = XPLMGetDataf(speedRef);
                landingData.ias = XPLMGetDataf(iasRef);
                landingData.windSpeed = XPLMGetDataf(windSpeedRef);
                landingData.windDir = XPLMGetDataf(windDirRef) + XPLMGetDataf(declinationRef);
                landingData.heading = XPLMGetDataf(headingRef);
                
                createEventWindow();
            }
            remainingShowTime = 60.0f;
        }
    }
    
    lastVSpeed = XPLMGetDataf(vertSpeedRef);
    lastState = state;
    return 0.05f;
}

