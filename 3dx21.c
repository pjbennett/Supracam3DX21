
/******************************************************************************
**  Supracam 2DX System  -  Main Development File                            **
**                                                                           **
**  Original Date: February 18, 2019                                             **
**  Author: PJ Bennett                                                       **
**                                                                           **
**  Module Description:                                                      **
**  This is the main module for the motion control system.  It includes      **
**  the main gtk glade file calls, motion data thread and cairo graphics.    **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
******************************************************************************/

//18-FEB-2019	v1.01	Initial version based on ELX-217a.c
//18-FEB-2019 	v1.02	Fixed joystick settings, added brake engage message
//18-FEB-2019	v1.03	Fixed cm per motor rev (measured at Duke). Joystick sensitivity increased

//04-MAR-2019	v1.06	Fixed thread time display, added jog functions, new glade, new title bar
//05-MAR-2019	v1.07	*** in work ***


#define BUILD_NUMBER "21.01.01"
#define GLADE_FILE_NAME "3dx21.glade"
#define CCS_FILE_NAME "3dx21.css"

#define _GNU_SOURCE
#include <gtk/gtk.h>
#include <stdio.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <gdk/gdk.h>
#include <pthread.h>
#include <sys/types.h>
#include <syscall.h>
#include <sched.h>
#include <glib.h> //for ini file functions
#include <glib/gprintf.h> 

//system hardware constant defines
//--------------------------------
//Drum/Line calcs
//8.125 diam of line
//76.70ft for 36 wraps measured
//76.57ft for 36 wraps calculated - using measured value
#define METERS_PER_DRUM_WRAP 0.64956
#define MOTOR_TO_DRUM_RATIO 3
#define METERS_PER_MOTOR_REV 0.21652
#define ENCODER_COUNTS_PER_REV 10000.0 //ten thousand - 2DX reels
#define CM_PER_ENCODER_COUNT 0.0013025 //use double precision
//#define CM_PER_MOTOR_REV 21.652 //AVATAR reels - adjust as needed to reduce position drift over time
#define CM_PER_MOTOR_REV 13.025 //2DX reels - adjust as needed to reduce position drift over time
//#define CM_PER_FRAME_TO_RPM 83.13328 //(cm/33.33ms)(1rev/21.652cm)(60000ms/1min)
#define CM_PER_FRAME_TO_RPM 69.1047992 //(cm/66.66ms)(1rev/13.025cm)(60000ms/1min)
#define CPF_TO_MPS 0.30 //converting centimeters per frame to meters per second

//software defines
#define WINCH_01 0
#define WINCH_02 1
#define WINCH_03 2
#define WINCH_04 3

#define CONFIG_FILE_NAME "3dx21.ini"
#define JOY_DEV "/dev/input/js0"

#define AVGCOUNT 300
#define AVGCOUNT2 180
#define FRAMECOUNT 3600

//moved to ini file
//#define SER_PORT_1 "/dev/ttyB15P0"
//#define SER_PORT_2 "/dev/ttyB15P1"
//#define SER_PORT_3 "/dev/ttyB15P2"
//#define SER_PORT_4 "/dev/ttyB15P3"
//#define USB_SER_PORT_1 "/dev/ttyUSB0"

#define ON 1
#define OFF 0

#define ORG01 255/255.0, 50/255.0, 0/255.0, 1.0 
#define YEL01 255/255.0, 174/255.0, 0/255.0, 1.0 
#define YEL02 125/255.0, 87/255.0, 0/255.0, 1.0
#define YEL03 125/255.0, 87/255.0, 0/255.0, 0.5
#define BLU01 0/255.0, 185/255.0, 240/255.0, 1.0
#define BLU02 0/255.0, 90/255.0, 124/255.0, 1.0
#define RED01 200/255.0, 0/255.0, 0/255.0, 1.0
#define RED02 100/255.0, 0/255.0, 0/255.0, 1.0

#define RED 255/255.0, 0/255.0, 0/255.0, 1.0
#define GRN 0/255.0, 255/255.0, 0/255.0, 1.0
#define BLU 0/255.0, 0/255.0, 255/255.0, 1.0
#define BLK 0/255.0, 0/255.0, 0/255.0, 1.0

#define EXIT_YES 1
#define EXIT_NO 2

#define WINCH_TELEMETRY_SPEED_X 195
#define WINCH_TELEMETRY_SPEED_Y 120
#define WINCH_TELEMETRY_LOAD_X 363
#define WINCH_TELEMETRY_LOAD_Y 120
#define WINCH_TELEMETRY_POSITION_X 531
#define WINCH_TELEMETRY_POSITION_Y 132
#define WINCH_TELEMETRY_CHART_X 112
#define WINCH_TELEMETRY_CHART_Y 155

#define MP_SCALE 40

#define PTR_Y_OFFSET 50.0 //in cm
#define FLASH_TOGGLE_RATE 10

#define OK 0
#define ERROR 1

//#define JOYSTICK_SMOOTHING 0.1
#define JOYSTICK_SMOOTHING 0.4
#define BOUNDARY_SMOOTHING 300.0
#define BOUNDARY_JOYRESET 20.0

#define MODE_LOCKED 1
#define MODE_AUTO 2
#define MODE_MANUAL 3

#define GOTOSTART_READY 1
#define GOTOSTART_MOVING 2
#define GOTOSTART_XMOVE 3
#define GOTOSTART_YMOVE 4
#define GOTOSTART_ZMOVE 5
#define GOTOSTART_ATSTART 6
#define GOTOSTART_MAXSPEED 5.0 //cm per frame
#define GOTOSTART_ATPOSITION 1.0 //cm - at position if within X cm +/-
#define GOTOSTART_BOUNDARY 100.0 //cm

#define PLAYBACK_MODE_STOP 1
#define PLAYBACK_MODE_PLAY 2
#define PLAYBACK_MODE_PAUSE 3
#define PLAYBACK_MODE_REWIND 4
#define PLAYBACK_MODE_GOTOSTART 5
#define PLAYBACK_MODE_DECEL 6

#define SPD_CHART_MULT -30

#define JOG_OFF 0
#define JOG_IN 1
#define JOG_OUT 2



//function prototypes
void on_btn_hello_clicked();
gboolean timeoutFunction (gpointer data);
gboolean on_joystick_change (GIOChannel *source, GIOCondition condition, gpointer data);
int16_t getMotorDataValue (int startByte);
int setupSerialPort (int winch, char* portName);
int setupUSBSerialPort (char* portName);
void updateTimeCode (void);
void* motorControllerThread (void *arg);
void readMotionFile (char *filename);
void appendMotionData (void);
void drawTelemetryChart (cairo_t *cr, int wnum, int xpos, int ypos);
void drawSpeedGauge (cairo_t *cr, int wnum, int xpos, int ypos);
void drawLoadGauge (cairo_t *cr, int wnum, int xpos, int ypos);
void drawPositionGauge (cairo_t *cr, int wnum, int xpos, int ypos);
void getWinchStatus (int wnum);
void on_btnValidate_clicked (void);
float limitf(float value, float min, float max);
int limit_int(int value, int min, int max);


//global variables
GKeyFile *gkfd; //glib ini file parser file descriptor

GtkWindow *g_mainwindow;
GtkWidget *g_mainfixed;

GtkWidget *g_drawarea;
GtkWidget *g_drawarea2;
GtkWidget *g_drawgauge1;
GtkWidget *g_drawgauge2;
GtkWidget *g_drawgauge3;
GtkWidget *g_drawgauge4;

GtkWidget *g_lbl_count;
GtkWidget *g_lbl_serdata;
GtkWidget *g_lbl_joyvalue;

GtkLabel *g_lblDebug1;
GtkLabel *g_lblDebug2;
GtkLabel *g_lblDebug3;
GtkLabel *g_lblDebug4;


GtkLabel *g_lbl_speedValue;
GtkLabel *g_lbl_loadValue;
GtkLabel *g_lbl_positionValue;
GtkLabel *g_lbl_serdatarcv0;
GtkLabel *g_lbl_serdatarcv1;
GtkLabel *g_lbl_serdatarcv2;
GtkLabel *g_lbl_serdatarcv3;

GtkLevelBar *g_lvl_speed;
GtkLevelBar *g_lvl_load;


GtkWidget *g_pbar_speed;
GtkProgressBar *g_pbar1;
GtkProgressBar *g_pbar2;

GtkListStore *g_motionDataStore;
GtkTreeIter g_treeIter;

GtkFileChooser *g_filechooser;
GtkTextView *g_txtStats;

GtkTextBuffer *g_txtBuffer;

GtkWidget *g_drawPTR;

GtkButton *g_btnPlayBackStop;
GtkButton *g_btnPlayBackPlay;

GtkWidget *g_drawFloor;
GtkWidget *g_drawElevation;
GtkWidget *g_drawXYZ;
GtkWidget *g_drawStatus;
GtkWidget *g_drawServoStatus;
GtkWidget *g_drawAlarmStatus;
GtkWidget *g_drawSpeedChart;

GtkEntry *g_shiftFrame;

GtkWidget *g_dlgExit;
GtkEntry *g_shiftX;
GtkEntry *g_shiftY;
GtkEntry *g_shiftZ;
GtkLabel *g_lblEntry;

GtkWidget *g_dlgSurvey;
GtkEntry *g_setSurveyP1X;
GtkEntry *g_setSurveyP1Y;
GtkEntry *g_setSurveyP1Z;
GtkEntry *g_setSurveyP2X;
GtkEntry *g_setSurveyP2Y;
GtkEntry *g_setSurveyP2Z;
GtkEntry *g_setSurveyP3X;
GtkEntry *g_setSurveyP3Y;
GtkEntry *g_setSurveyP3Z;
GtkEntry *g_setSurveyP4X;
GtkEntry *g_setSurveyP4Y;
GtkEntry *g_setSurveyP4Z;

GtkWidget *g_dlgCurrentPosition;
GtkEntry *g_setCurrentPositionX;
GtkEntry *g_setCurrentPositionY;
GtkEntry *g_setCurrentPositionZ;
//GtkLabel *g_lblSetCurrentPosition;

GtkLabel *g_lblModeStatus1;
GtkLabel *g_lblModeStatus2;
GtkLabel *g_lblPlayBackStatus;

GtkWidget *g_dlgBoundary;
GtkEntry *g_setBoundaryXmin;
GtkEntry *g_setBoundaryXmax;
GtkEntry *g_setBoundaryYmin;
GtkEntry *g_setBoundaryYmax;
GtkEntry *g_setBoundaryZmin;
GtkEntry *g_setBoundaryZmax;

GtkButton *g_btnFloor;

GtkComboBox *g_cbPlaybackSpeed;

GtkLabel *g_lblJogWinch1;
GtkComboBox *g_cbJogSpeed;

GtkLabel *g_lblPosition1;
GtkLabel *g_lblPosition2;
GtkLabel *g_lblPosition3;
GtkLabel *g_lblPosition4;




char commPortWinch1[80];
char commPortWinch2[80];
char commPortWinch3[80];
char commPortWinch4[80];
char commPortPTR[80];

static const double dash1[] = {6.0, 6.0}; //on,off
static const double dash2[] = {3.0, 6.0};

float term1, term2, term3;

int joyValue1;
int joyValue2;

int ser_fd;

int serialToggle = 0;
char readBuffer[80];
int bytesRead = 0;
char strMisc[80];
char strReceivedBuffer[80];
char strReceivedDisplayBuffer[4][80];
char strWriteBuffer[80];
char strTempBuffer[80];
char strUSBBuffer[80];

char strEntryBuffer[80];


int dw, dh, dx, dy, dyp;
int dir = 1;
int gauge_val;

clock_t starttime, endtime;
double cpu_time_used;
double cpu_time_used_max;
int cpu_time_max_count;

struct timeval starttv, endtv;
double elapsedtime;

float avgElapTimeValue[1000];
float avgElapTime;

int timeCount1;

struct Boundary {
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float zmin;
	float zmax;
	int floor;
	float yminSaved;
	//new boundary points (based on pulley points)
	float p1x;
	float p1z;
	float p2x;
	float p2z;
	float p3x;
	float p3z;
	float p4x;
	float p4z;
	};
struct Boundary systemBoundary;

struct Survey {
	float xLocation;
	float yLocation;
	float zLocation;
	};
struct Survey pulleyLocation[4];

struct SysteStatusStruct {
	int Master;
	int Comms;
	int Servo;
	int Alarm;
	int Moving;
	int PositionChange;
	};
struct SysteStatusStruct systemStatus;

struct WinchStatusStruct {
	int Comms;
	int Brake;
	int Servo;
	int Alarm;
	int AlarmCounter;
	int Overload;
	int VelocityReached;
	};
struct WinchStatusStruct winchStatus[4];

struct Winch {
	int speedCommand; //0-3000rpm	
	int speedCommandPrev; //0-3000rpm	
	int speedActual; //0-3000rpm
	float motorLoad; // 0-300%
	float motorLoadMax; //0-300%
	float PositionActual; //in revolutions
	float PositionCommand; //in revolutions
	float PositionError; //in Rev
	float PositionErrorMax; //in Rev
	float LineLength; //line length in cm
	float PositionCurr; //current position for frame
	float PositionPrev; //prev position for frame
	float PositionNew; //new position for frame
	float LineLengthAtZero; //base line length at zero point
	float LineLengthFromZero; //current line length from zero point
	float LineLengthPrev;
	float LineLengthChange;
	int JogStatus; //sets jogging in/out/off status
	float EncoderOffset;
	float LLoffset;
	float PAoffset;
	};
struct Winch winchData[4];

int winchChartSpeedData [4][610];
int winchChartSpeedCommandData [4][610];
int winchChartLoadData [4][610];

int threadChartData [650];

struct MotionDataStruct {
	int frameNumber;
	char timecode[20];
	float xPosition;
	float yPosition;
	float zPosition;
	float panRotation;
	float panMultiTurn;
	float tiltRotation;
	float rollRotation;
	};
struct MotionDataStruct motionData[9000]; //5 min of 30fps data
int motionDataFrames;

struct MotionManualStruct {
	int frameNumber;
	char timecode[20];
	float xPosition;
	float yPosition;
	float zPosition;
	float panMultiTurn;
	float panRotation;
	float tiltRotation;
	float rollRotation;
	};
struct MotionManualStruct motionManual; //manual position

struct MotionCommandStruct {
	int frameNumber;
	char timecode[20];
	float xPosition;
	float yPosition;
	float zPosition;
	float panMultiTurn;
	float panRotation;
	float tiltRotation;
	float rollRotation;
	};
struct MotionCommandStruct motionCommand; //commanded Position
struct MotionCommandStruct motionCommandPrev; //commanded Position


int mdcount;


double dsin;

struct {
  cairo_surface_t *image;
} glob;


struct timeval loopstart, loopend;
struct timeval handlerstart, handlerend;
struct timeval realtime, realtimecurrent;
double delta, deltaHandler;
double deltamax, deltamin;
long test1;
struct timespec sleepTimer;

struct timeval threadStart, threadEnd;
double threadDelta;

struct timeval threadStart2, threadEnd2;
double threadDelta2;


struct timeval sig2Start, sig2End;
double sig2Delta;

double avgTime;
double avgTimeValue[1000];
int avgTimeCount;

double avgTime2;
double avgTimeValue2[1000];
int avgTimeCount2;


long frameCount;
double totalTime;

int usbPortFD ; //USB port
//int port01_fd, port02_fd, port03_fd, port04_fd;  //PCI-1612 4 port card

struct termios SerialPortSettings;
char writeBuffer[80];
int bytesWritten = 0;

int tc_hr, tc_min, tc_sec, tc_frames;

int frameRateToggle;
int togglePTRplay;

int toggleMoveToZero;

int chartTimeCounter;
int chartTimeSeconds;

int testCounter = 0;

pthread_t tid;

int commErrorCount[4];
int commErrorCountTotal[4];
float positionError = 0;
float positionErrorMax = 0;

int motionDataFileValidated = 0;

int flashToggle;
int flashToggleCount;

//int winchCommStatus[4];
int serialPortFD[4];

float xjoymoveCommand;
float xjoymoveSmoothed;
float xPositionCurrent;
float xPositionPrev;
float yjoymoveCommand;
float yjoymoveSmoothed;
float yPositionCurrent;
float yPositionPrev;
float zjoymoveCommand;
float zjoymoveSmoothed;
float zPositionCurrent;
float zPositionPrev;
float pjoymoveCommand;
float pjoymoveSmoothed;

float xPositionStartUp;
float yPositionStartUp;
float zPositionStartUp;

int setCurrentPositionLockout;
int setServoOnOff;
int sendServoOnOff;
int sendServoOnOffCounter;

int sendAlarmReset;
int sendAlarmResetCounter;

int systemMode;
int gotoStartMode;
int gotoStartAtX;
int gotoStartAtY;
int gotoStartAtZ;

float gotoStartXSpeed;
float gotoStartYSpeed;
float gotoStartZSpeed;
float newSpeed;


int playbackMode;
int systemAtPlayPosition;
int systemAtPlayBackEnd;

int playbackDecelCount;
int playbackDecelValue;
float playbackDecelDiff;

float playbackDecelDiffX;
float playbackDecelDiffY;
float playbackDecelDiffZ;


float actualLLC[4];
float diffLLC[4];

float maxDiffLLC[4];
float cumultiveDiffLLC[4];

int panSendCounter;

int rcvDataLRC[4];
int calcDataLRC[4];
char strDataLRC[10];
int DataLRCcounter;

int startFrame = 1;
int pathWithinBoundary;

int playbackSpeedPercentage = 100; //in percentage
int playbackSpeedRate;
int playbackSpeedFrameCounter;

int chartZoom = 4;
float pan360Previous;
float pan360Current;
float panInputValue;
float pan360Value;
int pan360Rev;
float panMultiTurnValue;
float panManualMultiTurnValue;

float joytest1;
int buffNumBytes[4];

int jogSpeed = 1;

float LLcm[5];
float PAcm[5];
float DFcm[5];



//---------- TEST VARIABLES -----------
#define WINCH_IGNORE_ALL 	100
#define WINCH_IGNORE_ALL_BUT_1 	101
#define WINCH_IGNORE_ALL_BUT_2 	102
#define WINCH_IGNORE_ALL_BUT_3 	103
#define WINCH_IGNORE_ALL_BUT_4 	104

int simulationTest = TRUE;

//-------------------------------------


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

int main (int argc, char *argv[])
{
	int i;
	int joy_fd;
	int num_of_axis=0, num_of_buttons=0;
	char name_of_joystick[80];

	glob.image = cairo_image_surface_create_from_png("rpm.png");

	gettimeofday(&realtimecurrent, NULL);


	printf("\n\nProgram Start...\n");

	//----------------- Read Config File ---------------
	gkfd = g_key_file_new();
	if (!g_key_file_load_from_file(gkfd,CONFIG_FILE_NAME,G_KEY_FILE_NONE, NULL))
	{
		printf("could not read config file %s\n",CONFIG_FILE_NAME);
		return -1;
	}
	
	//commPortWinch1 = g_key_file_get_string (gkfd, "commPorts","commPortWinch1",NULL);
	sprintf(commPortWinch1, "%s", g_key_file_get_string (gkfd, "commPorts","commPortWinch1",NULL));
	sprintf(commPortWinch2, "%s", g_key_file_get_string (gkfd, "commPorts","commPortWinch2",NULL));
	sprintf(commPortWinch3, "%s", g_key_file_get_string (gkfd, "commPorts","commPortWinch3",NULL));
	sprintf(commPortWinch4, "%s", g_key_file_get_string (gkfd, "commPorts","commPortWinch4",NULL));
	sprintf(commPortPTR, "%s", g_key_file_get_string (gkfd, "commPorts","commPortPTR",NULL));
	
	pulleyLocation[0].xLocation = g_key_file_get_double (gkfd,"survey","surveyP1X",NULL);
	pulleyLocation[0].yLocation = g_key_file_get_double (gkfd,"survey","surveyP1Y",NULL);;
	pulleyLocation[0].zLocation = g_key_file_get_double (gkfd,"survey","surveyP1Z",NULL);;

	pulleyLocation[1].xLocation = g_key_file_get_double (gkfd,"survey","surveyP2X",NULL);;
	pulleyLocation[1].yLocation = g_key_file_get_double (gkfd,"survey","surveyP2Y",NULL);;
	pulleyLocation[1].zLocation = g_key_file_get_double (gkfd,"survey","surveyP2Z",NULL);;

	pulleyLocation[2].xLocation = g_key_file_get_double (gkfd,"survey","surveyP3X",NULL);;
	pulleyLocation[2].yLocation = g_key_file_get_double (gkfd,"survey","surveyP3Y",NULL);;
	pulleyLocation[2].zLocation = g_key_file_get_double (gkfd,"survey","surveyP3Z",NULL);;

	pulleyLocation[3].xLocation = g_key_file_get_double (gkfd,"survey","surveyP4X",NULL);;
	pulleyLocation[3].yLocation = g_key_file_get_double (gkfd,"survey","surveyP4Y",NULL);;
	pulleyLocation[3].zLocation = g_key_file_get_double (gkfd,"survey","surveyP4Z",NULL);;
		
	xPositionStartUp = g_key_file_get_double (gkfd,"savedPosition","savedPositionX",NULL);
	yPositionStartUp = g_key_file_get_double (gkfd,"savedPosition","savedPositionY",NULL);
	zPositionStartUp = g_key_file_get_double (gkfd,"savedPosition","savedPositionZ",NULL);
	
	xPositionCurrent = xPositionStartUp;
	yPositionCurrent = yPositionStartUp;
	zPositionCurrent = zPositionStartUp;

	motionCommand.xPosition = xPositionStartUp;
	motionCommand.yPosition = yPositionStartUp;
	motionCommand.zPosition = zPositionStartUp;
	
	systemBoundary.xmin = g_key_file_get_double (gkfd,"boundary","boundaryXmin",NULL);
	systemBoundary.xmax = g_key_file_get_double (gkfd,"boundary","boundaryXmax",NULL);
	systemBoundary.ymin = g_key_file_get_double (gkfd,"boundary","boundaryYmin",NULL);
	systemBoundary.ymax = g_key_file_get_double (gkfd,"boundary","boundaryYmax",NULL);
	systemBoundary.zmin = g_key_file_get_double (gkfd,"boundary","boundaryZmin",NULL);
	systemBoundary.zmax = g_key_file_get_double (gkfd,"boundary","boundaryZmax",NULL);

	systemBoundary.floor = ON;
	systemBoundary.yminSaved = systemBoundary.ymin;

	systemBoundary.p1x = -2000;
	systemBoundary.p1z = 5000;
	systemBoundary.p2x = -4000;
	systemBoundary.p2z = -5000;
	systemBoundary.p3x = 4000;
	systemBoundary.p3z = -5000;
	systemBoundary.p4x = 2000;
	systemBoundary.p4z = 5000;

	//END ----------------- Read Config File ---------------


	//----------- global inits ---------------
	winchStatus[0].Comms = ERROR;
	winchStatus[1].Comms = ERROR;
	winchStatus[2].Comms = ERROR;
	winchStatus[3].Comms = ERROR;
	systemMode = MODE_LOCKED;
	//calculate line lengths from zero
	for (i = 0; i < 4; i++)
	{
		term1 = pulleyLocation[i].xLocation;
		term2 = pulleyLocation[i].yLocation;
		term3 = pulleyLocation[i].zLocation; //starting height
		winchData[i].LineLengthAtZero = sqrtf((term1*term1)+(term2*term2)+(term3*term3));
	}
	
	//---------- serial port -------------
	usbPortFD = setupUSBSerialPort (commPortPTR);
	serialPortFD[0] = setupSerialPort (1,commPortWinch1);
	serialPortFD[1] = setupSerialPort (2,commPortWinch2);
	serialPortFD[2] = setupSerialPort (3,commPortWinch3);
	serialPortFD[3] = setupSerialPort (4,commPortWinch4);

	//------------- joystick setup ----------------
	if ((joy_fd = open(JOY_DEV, (O_RDWR | O_NOCTTY | O_NDELAY))) == -1)
	{
		printf("\nCan't open joystick\n");
		//return -1;
	}
	else
	{
	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	printf("\n\nJoystick detected\n Name: %s\n Axis: %d\n Buttons: %d\n"
			, name_of_joystick
			, num_of_axis
			, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */
	}


    //---- setup and launch motion control thread ---

    pthread_attr_t threadAttr;
    struct sched_param param;
    int err;

    err = pthread_attr_init (&threadAttr);
    if (err !=0) {
		printf("error init attr\n");
		return -1;
	}

    err = pthread_attr_setschedpolicy (&threadAttr, SCHED_FIFO);
    if (err !=0) {
		printf("error set sched policy attr\n");
		return -1;
	}

	err=pthread_attr_getschedparam(&threadAttr, &param);
    if (err !=0) {
		printf("error %d - getting sched param\n", err);
		return -1;
	}

    param.sched_priority = 60;
    err = pthread_attr_setschedparam (&threadAttr, &param);
    if (err !=0) {
		printf("error %d - setting sched param\n", err);
		return -1;
	}

	//err = pthread_attr_setinheritsched(&threadAttr, PTHREAD_EXPLICIT_SCHED);
    if (err !=0) {
		printf("error setting sched inherit\n");
		return -1;
	}


    //err = pthread_create(&tid,&threadAttr,&myThread, NULL);
    if (err !=0) {
		printf("error %d - thread create\n", err);
		return -1;
    }


    //create thread
    err = pthread_create(&tid,&threadAttr,&motorControllerThread, NULL);
    if (err !=0) {
		printf("error %d - thread create\n", err);
		return -1;
	}

	//name thread
    err = pthread_setname_np(tid, "ELX-MOTION"); //max 15 char
    if (err !=0) {
		printf("error %d - set thread name\n", err);
		return -1;
    }


	//------------ GTK -------------------//
  GtkBuilder *builder;
  GtkWidget *window;

  gtk_init(&argc, &argv);

  builder = gtk_builder_new();
  gtk_builder_add_from_file (builder, GLADE_FILE_NAME, NULL);

  window = GTK_WIDGET (gtk_builder_get_object(builder, "window_main"));
  gtk_builder_connect_signals(builder, NULL);

	gtk_widget_set_name(GTK_WIDGET(window), "myWindow_Main");

  g_mainfixed = GTK_WIDGET(gtk_builder_get_object(builder, "MainFixed"));


  
  //get pointers to the labels
  g_lblDebug1 = GTK_LABEL(gtk_builder_get_object(builder, "lblDebug1"));
  g_lblDebug2 = GTK_LABEL(gtk_builder_get_object(builder, "lblDebug2"));
  g_lblDebug3 = GTK_LABEL(gtk_builder_get_object(builder, "lblDebug3"));
  g_lblDebug4 = GTK_LABEL(gtk_builder_get_object(builder, "lblDebug4"));

  g_lbl_count = GTK_WIDGET(gtk_builder_get_object(builder, "lbl_count"));
  g_lbl_serdata = GTK_WIDGET(gtk_builder_get_object(builder, "lblSerData"));
  g_lbl_joyvalue = GTK_WIDGET(gtk_builder_get_object(builder, "lblJoyValue"));
  g_pbar1 = GTK_PROGRESS_BAR(gtk_builder_get_object(builder, "pbarSpeed"));
  g_pbar2 = GTK_PROGRESS_BAR(gtk_builder_get_object(builder, "pbar2"));
  g_lbl_speedValue = GTK_LABEL(gtk_builder_get_object(builder, "lblSpeed"));
  g_lbl_loadValue = GTK_LABEL(gtk_builder_get_object(builder, "lblLoad"));
  g_lbl_positionValue = GTK_LABEL(gtk_builder_get_object(builder, "lblPosition"));
  g_lvl_load = GTK_LEVEL_BAR(gtk_builder_get_object(builder, "lvlLoad"));
  g_lvl_speed = GTK_LEVEL_BAR(gtk_builder_get_object(builder, "lvlSpeed"));
  g_lbl_serdatarcv0 = GTK_LABEL(gtk_builder_get_object(builder, "lblSerDataRcv0"));
  g_lbl_serdatarcv1 = GTK_LABEL(gtk_builder_get_object(builder, "lblSerDataRcv1"));
  g_lbl_serdatarcv2 = GTK_LABEL(gtk_builder_get_object(builder, "lblSerDataRcv2"));
  g_lbl_serdatarcv3 = GTK_LABEL(gtk_builder_get_object(builder, "lblSerDataRcv3"));

  g_drawarea = GTK_WIDGET(gtk_builder_get_object(builder, "drawingarea1"));
  g_drawarea2 = GTK_WIDGET(gtk_builder_get_object(builder, "drawingarea2"));
  g_drawgauge1 = GTK_WIDGET(gtk_builder_get_object(builder, "drawWinch1"));
  g_drawgauge2 = GTK_WIDGET(gtk_builder_get_object(builder, "drawWinch2"));
  g_drawgauge3 = GTK_WIDGET(gtk_builder_get_object(builder, "drawWinch3"));
  g_drawgauge4 = GTK_WIDGET(gtk_builder_get_object(builder, "drawWinch4"));

  g_motionDataStore = GTK_LIST_STORE(gtk_builder_get_object(builder, "liststore1"));
  g_filechooser = GTK_FILE_CHOOSER(gtk_builder_get_object(builder, "filechooserbutton1"));
  g_txtStats = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "txtStats"));
  g_txtBuffer = gtk_text_buffer_new(NULL);
  g_drawPTR = GTK_WIDGET(gtk_builder_get_object(builder, "ptrDrawingArea"));
  g_drawFloor = GTK_WIDGET(gtk_builder_get_object(builder, "floorDrawArea"));
  g_drawElevation = GTK_WIDGET(gtk_builder_get_object(builder, "elevationDrawArea"));

  g_shiftFrame = GTK_ENTRY(gtk_builder_get_object(builder, "entShiftFrame"));

  g_shiftX = GTK_ENTRY(gtk_builder_get_object(builder, "entShiftX"));
  g_shiftY = GTK_ENTRY(gtk_builder_get_object(builder, "entShiftY"));
  g_shiftZ = GTK_ENTRY(gtk_builder_get_object(builder, "entShiftZ"));
  g_lblEntry = GTK_LABEL(gtk_builder_get_object(builder, "lblEntry"));

  g_dlgExit = GTK_WIDGET(gtk_builder_get_object(builder, "dlgExit"));



  g_dlgCurrentPosition = GTK_WIDGET(gtk_builder_get_object(builder, "dlgCurrentPosition"));
  g_setCurrentPositionX = GTK_ENTRY(gtk_builder_get_object(builder, "entCurrentPositionX"));
  g_setCurrentPositionY = GTK_ENTRY(gtk_builder_get_object(builder, "entCurrentPositionY"));
  g_setCurrentPositionZ = GTK_ENTRY(gtk_builder_get_object(builder, "entCurrentPositionZ"));
  //g_lblSetCurrentPosition = GTK_LABEL(gtk_builder_get_object(builder, "lblSetCurrentPosition"));


  g_drawXYZ = GTK_WIDGET(gtk_builder_get_object(builder, "areaXYZ"));
  g_drawStatus = GTK_WIDGET(gtk_builder_get_object(builder, "areaStatus"));
  g_drawServoStatus = GTK_WIDGET(gtk_builder_get_object(builder, "areaServoStatus"));
  g_drawAlarmStatus = GTK_WIDGET(gtk_builder_get_object(builder, "areaAlarmStatus"));
  //g_drawModeStatus = GTK_WIDGET(gtk_builder_get_object(builder, "areaModeStatus"));
  g_drawSpeedChart = GTK_WIDGET(gtk_builder_get_object(builder, "areaSpeedChart"));

  g_lblModeStatus1 = GTK_LABEL(gtk_builder_get_object(builder, "lblModeStatus1"));
  g_lblModeStatus2 = GTK_LABEL(gtk_builder_get_object(builder, "lblModeStatus2"));
  g_lblPlayBackStatus = GTK_LABEL(gtk_builder_get_object(builder, "lblPlayBackStatus"));


  g_dlgBoundary = GTK_WIDGET(gtk_builder_get_object(builder, "dlgBoundary"));
  g_setBoundaryXmin = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryXmin"));
  g_setBoundaryXmax = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryXmax"));
  g_setBoundaryYmin = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryYmin"));
  g_setBoundaryYmax = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryYmax"));
  g_setBoundaryZmin = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryZmin"));
  g_setBoundaryZmax = GTK_ENTRY(gtk_builder_get_object(builder, "entBoundaryZmax"));

  g_btnFloor = GTK_BUTTON(gtk_builder_get_object(builder, "btnFloorOnOff"));

  g_btnPlayBackStop = GTK_BUTTON(gtk_builder_get_object(builder, "btnPlayBackStop"));
  g_btnPlayBackPlay = GTK_BUTTON(gtk_builder_get_object(builder, "btnPlayBackPlay"));

  g_dlgSurvey = GTK_WIDGET(gtk_builder_get_object(builder, "dlgSurvey"));
  g_setSurveyP1X = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP1X"));
  g_setSurveyP1Y = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP1Y"));
  g_setSurveyP1Z = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP1Z"));
  g_setSurveyP2X = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP2X"));
  g_setSurveyP2Y = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP2Y"));
  g_setSurveyP2Z = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP2Z"));
  g_setSurveyP3X = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP3X"));
  g_setSurveyP3Y = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP3Y"));
  g_setSurveyP3Z = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP3Z"));
  g_setSurveyP4X = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP4X"));
  g_setSurveyP4Y = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP4Y"));
  g_setSurveyP4Z = GTK_ENTRY(gtk_builder_get_object(builder, "entSurveyP4Z"));

  g_cbPlaybackSpeed = GTK_COMBO_BOX(gtk_builder_get_object(builder, "cbPlaybackSpeed"));

  g_lblJogWinch1 = GTK_LABEL(gtk_builder_get_object(builder, "lblJogWinch1"));
  g_cbJogSpeed = GTK_COMBO_BOX(gtk_builder_get_object(builder, "cbJogSpeed"));

  g_lblPosition1 = GTK_LABEL(gtk_builder_get_object(builder, "lblPosition1"));
  g_lblPosition2 = GTK_LABEL(gtk_builder_get_object(builder, "lblPosition2"));
  g_lblPosition3 = GTK_LABEL(gtk_builder_get_object(builder, "lblPosition3"));
  g_lblPosition4 = GTK_LABEL(gtk_builder_get_object(builder, "lblPosition4"));

  gtk_widget_set_name(GTK_WIDGET(g_btnFloor), "myButton_floor");



  gdk_threads_add_timeout (50, timeoutFunction, NULL); //50ms between updates

	//add joystick io channel watch and callback
	GIOChannel *channel = g_io_channel_unix_new(joy_fd);
	g_io_channel_set_encoding(channel, NULL, NULL);
	g_io_add_watch(channel, G_IO_IN, on_joystick_change, NULL);


	/* ----------------- CSS --------------------------*/
	GtkCssProvider *provider = gtk_css_provider_new ();
	gtk_style_context_add_provider_for_screen (gdk_screen_get_default(), GTK_STYLE_PROVIDER (provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
	gtk_css_provider_load_from_path(provider, CCS_FILE_NAME, NULL);
	g_object_unref (provider);
	/* ------------------------------------------------*/




	//remove no longer needed objects
	g_object_unref(builder);
	cairo_surface_destroy(glob.image);

	gtk_window_fullscreen(GTK_WINDOW(window));
	gtk_widget_show(window);
	gtk_main();

	//--- application runs in gtk_main event loop ---


  	//upon exit print this status info
    printf("\nELX exited normally\n");

	return 0;
}
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

void* motorControllerThread (void *arg)
{
	//****** MAKE SURE THERE ARE NO GTK CALLS IN THIS THREAD *******
	
	//also, this thread runs about 10 times before the full app has 
	//had a chance to run through once, so delay any calcs and serial
	//transmissions in this thread for the first 60 frames (1 sec) 

	int i, waiting = 0;
	struct timeval waitTime;
	int16_t speedval;


	while (1)
	{
		//usleep(5000);
		frameCount++;
		updateTimeCode();
		usleep(500);
		waiting = 0;
		while (waiting == 0)
		{
			gettimeofday(&waitTime, NULL);
			if (threadStart2.tv_usec > waitTime.tv_usec) //take care of crossing into new second
				waitTime.tv_usec += 1000000;
			//if (waitTime.tv_usec > (threadStart2.tv_usec + 16661)) //was 16653 with generic
			if (waitTime.tv_usec > (threadStart2.tv_usec + 33332)) //was 16653 with generic
				waiting = 1;
		}

	if (frameCount > 30)
	{

	//send data request message	
	if (serialToggle == 1)
	{
		serialToggle = 0;
		sprintf(writeBuffer,":010300040006F2\r\n");
		for (i=0;i<4;i++)
		{
			tcflush(serialPortFD[i], TCIOFLUSH);  //flush serial port (get rid of previous cmd response)
			bytesWritten = write(serialPortFD[i],writeBuffer,strlen(writeBuffer));
		}
	}
	//read previous data request and then send command data message
	else
	{
		serialToggle = 1;
		
		for (i=0;i<4;i++)
		{
			sprintf(strReceivedDisplayBuffer[i], "Winch %d: waiting for data...", i+1);

			//read whatever is in the serial port buffer (how big is hardware buffer?)
			buffNumBytes[i] = 0;
			ioctl(serialPortFD[i], FIONREAD, &buffNumBytes[i]);
			//printf("port %d bytes available: %d\n", i, buffNumBytes[i]);
			//printf("Number of bytes = %d\n", bytes);
			bytesRead = 0;

			//check bytes available
			if (buffNumBytes[i] == 35) //35
			{
				bytesRead = read(serialPortFD[i], readBuffer, 35); //35
			}
			else if (buffNumBytes[i] > 35)
			{
				//bytesRead = read(serialPortFD[i], readBuffer, buffNumBytes[i]); //35
				//bytesRead = read(serialPortFD[i], readBuffer, 35); //35
				//readBuffer[bytesRead - 1] = 0; //set string terminator, delete \n at end of string
				sprintf(strReceivedDisplayBuffer[i], "Winch %d: Error - greater than 35 bytes rcvd", i+1);
				commErrorCount[i]++;
				commErrorCountTotal[i]++;
				//sprintf(strReceivedDisplayBuffer[i], "Winch %d: Comm 35+ Error", i+1);
				//tcflush(serialPortFD[i], TCIOFLUSH);  //flush serial port (get rid of previous cmd response)		
			}
			else if (buffNumBytes[i] <= 30)
			{
				commErrorCount[i]++;
				commErrorCountTotal[i]++;
				sprintf(strReceivedDisplayBuffer[i], "Winch %d: Error - less than 35 bytes rcvd", i+1);
			}

			//check read bytes (not just available bytes)
			//if (bytesRead < 0)
			//{
			//	commErrorCount[i]++;
			//	commErrorCountTotal[i]++;
			//	sprintf(strReceivedDisplayBuffer[i], "Winch %d: Comm Error - no data", i+1);
			//}

			if (commErrorCount[i] > 8)
			{
				winchStatus[i].Comms = ERROR;
				commErrorCount[i] = 100;
			}


			if (bytesRead > 0)
			{
				readBuffer[bytesRead - 1] = 0; //set string terminator, delete \n at end of string
				//printf("read %d: %s", bytesRead, readBuffer);
				//sprintf(strReceivedBuffer, "%s", readBuffer);
				
				sprintf(strReceivedDisplayBuffer[i], "Winch %d: %s", i+1, readBuffer);
				
				//=== calculate message checksum ===
				//read packet sent checksum
				strDataLRC[0] = readBuffer[31];
				strDataLRC[1] = readBuffer[32];
				strDataLRC[2] = 0; //string terminator
				rcvDataLRC[i] = (int16_t)strtol(strDataLRC, NULL, 16); //hex conversion
				//calculate packet checksum
				calcDataLRC[i] = 0;
				for (DataLRCcounter = 1; DataLRCcounter < 31; DataLRCcounter+=2)
				{					
					strDataLRC[0] = readBuffer[DataLRCcounter];
					strDataLRC[1] = readBuffer[DataLRCcounter+1];
					strDataLRC[2] = 0; //string terminator
					calcDataLRC[i] = calcDataLRC[i] + (int16_t)strtol(strDataLRC, NULL, 16); //hex conversion
				}
				calcDataLRC[i] = 0x10000 - calcDataLRC[i]; //twos compliment
				calcDataLRC[i] = 0x0000FF & calcDataLRC[i]; //mask for lower byte only


				//=== verify message packet checksum, if good decode and use data ===
				if (calcDataLRC[i] == rcvDataLRC[i])
				{
					//if good message then COMMS are ok, reset error count
					commErrorCount[i] = 0;
					winchStatus[i].Comms = OK;
					
					getWinchStatus(i); //update the winch status from rcvd data

					winchData[i].speedActual = getMotorDataValue (7);
					winchData[i].motorLoad = getMotorDataValue (15);
					if (fabs(winchData[i].motorLoad) > winchData[i].motorLoadMax)
					{
						winchData[i].motorLoadMax = fabs(winchData[i].motorLoad);
					}
					int16_t positionValue1 = getMotorDataValue (11);
					int16_t positionValue2 = getMotorDataValue (19);

					winchData[i].PositionActual = positionValue1 + ((float)positionValue2/10000.0);
					
					//winchData[i].PositionError = winchData[i].PositionActual - winchData[i].PositionCommand;
					//use speed error as better indicator of position error
//+++++++					winchData[i].PositionError = (float_t)(winchData[i].speedActual - winchData[i].speedCommand)/10.0; 

					if (fabs(winchData[i].PositionError) > winchData[i].PositionErrorMax)
					{
						winchData[i].PositionErrorMax = fabs(winchData[i].PositionError);
					}
				}
				else
				{
					commErrorCount[i]++;
					commErrorCountTotal[i]++;
				}
			}
		}
		
		
//test simulated data
		if (simulationTest == TRUE)
		{
			winchStatus[0].Comms = OK;
			winchStatus[1].Comms = OK;
			winchStatus[2].Comms = OK;
			winchStatus[3].Comms = OK;
			winchStatus[0].Alarm = OFF;
			winchStatus[1].Alarm = OFF;
			winchStatus[2].Alarm = OFF;
			winchStatus[3].Alarm = OFF;

		
			if (setServoOnOff == ON)
			{
				winchStatus[0].Servo = ON;
				winchStatus[1].Servo = ON;
				winchStatus[2].Servo = ON;
				winchStatus[3].Servo = ON;
				winchStatus[0].Brake = OFF;
				winchStatus[1].Brake = OFF;
				winchStatus[2].Brake = OFF;
				winchStatus[3].Brake = OFF;
			}
			else
			{
				winchStatus[0].Servo = OFF;
				winchStatus[1].Servo = OFF;
				winchStatus[2].Servo = OFF;
				winchStatus[3].Servo = OFF;
				winchStatus[0].Brake = ON;
				winchStatus[1].Brake = ON;
				winchStatus[2].Brake = ON;
				winchStatus[3].Brake = ON;
			}
		}
//test simulated data



		//check comms and update system status
		if ((winchStatus[0].Comms == OK) &&
			(winchStatus[1].Comms == OK) &&
			(winchStatus[2].Comms == OK) &&
			(winchStatus[3].Comms == OK))
		{
			systemStatus.Comms = OK;
		}
		else
		{
			systemStatus.Comms = ERROR;			
		}
			
		//check servos and update system status
		if ((winchStatus[0].Servo == ON) &&
			(winchStatus[1].Servo == ON) &&
			(winchStatus[2].Servo == ON) &&
			(winchStatus[3].Servo == ON))
		{
			systemStatus.Servo = OK;
		}
		else
		{
			systemStatus.Servo = ERROR;			
		}
		
		//check servos and update system status
		if ((winchStatus[0].Alarm == OFF) &&
			(winchStatus[1].Alarm == OFF) &&
			(winchStatus[2].Alarm == OFF) &&
			(winchStatus[3].Alarm == OFF))
		{
			systemStatus.Alarm = OK;
		}
		else
		{
			systemStatus.Alarm = ERROR;			
		}
		
		
		//servo off all winches and do not allow joystick inputs 
		//if there are any system errors
		if ((systemStatus.Comms == ERROR) || (systemStatus.Alarm == ERROR) || (systemStatus.PositionChange == ERROR))
		{
			//printf("Frame:  %05ld  Comms: %d  Alarm: %d\n", frameCount, systemStatus.Comms, systemStatus.Alarm);
			//constantly send servo off sequence
			if (sendServoOnOff == FALSE) //make sure previous servo off sequence is complete
			{
				setServoOnOff = OFF;
				sendServoOnOff = TRUE;
				sendServoOnOffCounter = 0;
			}			

			xjoymoveCommand = 0;
			yjoymoveCommand = 0;
			zjoymoveCommand = 0;
		}

		//servo off all winches and do not allow joystick inputs 
		//if there are any system errors
		if (systemStatus.Servo == ERROR)
		{
			//setServoOnOff = OFF;
			//sendServoOnOff = TRUE;
			xjoymoveCommand = 0;
			yjoymoveCommand = 0;
			zjoymoveCommand = 0;
		}
		
		if ((systemStatus.Alarm == OK) &&
			(systemStatus.Comms == OK) &&
			(systemStatus.PositionChange == OK) &&
			(systemStatus.Servo == OK))
		{
			systemStatus.Master = OK;
		}
		else
		{
			systemStatus.Master = ERROR;
			//systemMode = MODE_MANUAL;
		}
			
		if ((xPositionCurrent < motionData[mdcount].xPosition + 3.0) &&
			(xPositionCurrent > motionData[mdcount].xPosition - 3.0) &&
			(yPositionCurrent < motionData[mdcount].yPosition + 3.0) &&
			(yPositionCurrent > motionData[mdcount].yPosition - 3.0) &&
			(zPositionCurrent < motionData[mdcount].zPosition + 3.0) &&
			(zPositionCurrent > motionData[mdcount].zPosition - 3.0))
		{
			systemAtPlayPosition = TRUE;
		}
		else
		{
			systemAtPlayPosition = FALSE;			
		}

		//========== goto start mode ===================
		
		if ((systemMode == MODE_AUTO) && (playbackMode == PLAYBACK_MODE_GOTOSTART))
		{
			if (gotoStartMode == GOTOSTART_MOVING)
			{
				//goto Start X axis motion
				if (xPositionCurrent > motionData[startFrame].xPosition + GOTOSTART_ATPOSITION) //greater than, but not at position
				{
					if (gotoStartXSpeed < GOTOSTART_MAXSPEED)
						gotoStartXSpeed+= 0.05; //slowly increase speed
					if (xPositionCurrent < motionData[startFrame].xPosition + GOTOSTART_BOUNDARY) //inside decel boundary
					{
						newSpeed = gotoStartXSpeed * (abs(motionData[startFrame].xPosition - xPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.xPosition = motionCommand.xPosition - newSpeed;					
					}
					else
						motionCommand.xPosition = motionCommand.xPosition - gotoStartXSpeed;					
				}
				else if (xPositionCurrent < motionData[startFrame].xPosition - GOTOSTART_ATPOSITION) //less than, but not at position
				{
					if (gotoStartXSpeed < GOTOSTART_MAXSPEED)
						gotoStartXSpeed+= 0.05; //slowly increase speed
					if (xPositionCurrent > motionData[startFrame].xPosition - GOTOSTART_BOUNDARY) //inside decel boundary
					{
						newSpeed = gotoStartXSpeed * (abs(motionData[startFrame].xPosition - xPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.xPosition = motionCommand.xPosition + newSpeed;					
					}
					else
						motionCommand.xPosition = motionCommand.xPosition + gotoStartXSpeed;					
				}
				else
				{
					gotoStartAtX = TRUE;
					gotoStartXSpeed = 0;
				}
				

				//goto Start Y axis motion
				if (yPositionCurrent > motionData[startFrame].yPosition + GOTOSTART_ATPOSITION) //greater than, but not at position
				{
					if (gotoStartYSpeed < GOTOSTART_MAXSPEED)
						gotoStartYSpeed+= 0.05; //slowly increase speed
					if (yPositionCurrent < motionData[startFrame].yPosition + GOTOSTART_BOUNDARY) //inside decel boundary
					{
						newSpeed = gotoStartYSpeed * (abs(motionData[startFrame].yPosition - yPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.yPosition = motionCommand.yPosition - newSpeed;					
					}
					else
						motionCommand.yPosition = motionCommand.yPosition - gotoStartYSpeed;					
				}
				else if (yPositionCurrent < motionData[startFrame].yPosition - GOTOSTART_ATPOSITION) //less than, but not at position
				{
					if (gotoStartYSpeed < GOTOSTART_MAXSPEED)
						gotoStartYSpeed+= 0.05; //slowly increase speed
					if (yPositionCurrent > motionData[startFrame].yPosition - GOTOSTART_BOUNDARY) //inside boundary
					{
						newSpeed = gotoStartYSpeed * (abs(motionData[startFrame].yPosition - yPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.yPosition = motionCommand.yPosition + newSpeed;					
					}
					else
						motionCommand.yPosition = motionCommand.yPosition + gotoStartXSpeed;					
				}
				else
				{
					gotoStartAtY = TRUE;
					gotoStartYSpeed = 0;
				}

				//goto Start Z axis motion
				if (zPositionCurrent > motionData[startFrame].zPosition + GOTOSTART_ATPOSITION) //greater than, but not at position
				{
					if (gotoStartZSpeed < GOTOSTART_MAXSPEED)
						gotoStartZSpeed+= 0.05; //slowly increase speed
					if (zPositionCurrent < motionData[startFrame].zPosition + GOTOSTART_BOUNDARY) //inside boundary
					{
						newSpeed = gotoStartZSpeed * (abs(motionData[startFrame].zPosition - zPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.zPosition = motionCommand.zPosition - newSpeed;					
					}
					else
						motionCommand.zPosition = motionCommand.zPosition - gotoStartZSpeed;					
				}
				else if (zPositionCurrent < motionData[startFrame].zPosition - GOTOSTART_ATPOSITION) //less than, but not at position
				{
					if (gotoStartZSpeed < GOTOSTART_MAXSPEED)
						gotoStartZSpeed+= 0.05; //slowly increase speed
					if (zPositionCurrent > motionData[startFrame].zPosition - GOTOSTART_BOUNDARY) //inside boundary
					{
						newSpeed = gotoStartZSpeed * (abs(motionData[startFrame].zPosition - zPositionCurrent)/GOTOSTART_BOUNDARY);
						motionCommand.zPosition = motionCommand.zPosition + newSpeed;					
					}
					else
						motionCommand.zPosition = motionCommand.zPosition + gotoStartZSpeed;					
				}
				else
				{
					gotoStartAtZ = TRUE;
					gotoStartZSpeed = 0;
				}


			}
			if ((gotoStartAtX == TRUE) && (gotoStartAtY == TRUE) && (gotoStartAtZ == TRUE))
			{
				gotoStartMode = GOTOSTART_ATSTART;
				playbackMode = PLAYBACK_MODE_STOP;
			}
			
			//to keep manual position in sync
			motionManual.xPosition = motionCommand.xPosition;
			motionManual.yPosition = motionCommand.yPosition;
			motionManual.zPosition = motionCommand.zPosition;
			xPositionCurrent = motionCommand.xPosition;
			yPositionCurrent = motionCommand.yPosition;
			zPositionCurrent = motionCommand.zPosition;

		}


		//========== manual mode ===================

		if (systemMode == MODE_MANUAL)
		{
		mdcount = startFrame-1; //mdcount is zero based
		float moveBoundary;

		//-------- X --------
		//limit xPositionCurrent changes
		if (xjoymoveCommand < (xjoymoveSmoothed - JOYSTICK_SMOOTHING))
		{
			xjoymoveSmoothed-=JOYSTICK_SMOOTHING;
		}
		if (xjoymoveCommand > (xjoymoveSmoothed + JOYSTICK_SMOOTHING))
		{
			xjoymoveSmoothed+=JOYSTICK_SMOOTHING;
		}
		if (xjoymoveCommand == 0)
		{
			if ((xjoymoveSmoothed <= JOYSTICK_SMOOTHING) && (xjoymoveSmoothed >= -JOYSTICK_SMOOTHING))
			{
				xjoymoveSmoothed = 0;
			}
		}
		
		//boundary smoothing
		if (xjoymoveSmoothed > 0)
		{		
			if (xPositionCurrent > systemBoundary.xmax - BOUNDARY_SMOOTHING)
			{
				moveBoundary = xjoymoveSmoothed * ((systemBoundary.xmax - xPositionCurrent)/BOUNDARY_SMOOTHING);
				xPositionCurrent = xPositionCurrent + moveBoundary;
			}
			else
			{
				xPositionCurrent = xPositionCurrent + xjoymoveSmoothed;
			}
		}
		
		if (xjoymoveSmoothed <= 0)
		{
			if (xPositionCurrent < systemBoundary.xmin + BOUNDARY_SMOOTHING)
			{
				moveBoundary = xjoymoveSmoothed * ((systemBoundary.xmin - xPositionCurrent)/BOUNDARY_SMOOTHING) * -1.0;
				xPositionCurrent = xPositionCurrent + moveBoundary;
			}
			else
			{
				xPositionCurrent = xPositionCurrent + xjoymoveSmoothed;
			}
		}
		
		//double check to make sure position can't be outside boundary
		if (xPositionCurrent >= systemBoundary.xmax)
		{
			xPositionCurrent = systemBoundary.xmax;
		}
		if (xPositionCurrent <= systemBoundary.xmin)
		{
			xPositionCurrent = systemBoundary.xmin;
		}
		
		//reset xjoymovesmoothed if at (or near) boundary
		if ((xPositionCurrent >= systemBoundary.xmax - BOUNDARY_JOYRESET) && (xjoymoveCommand == 0)) 
		{
			xjoymoveSmoothed = 0;
		}
		if ((xPositionCurrent <= systemBoundary.xmin + BOUNDARY_JOYRESET) && (xjoymoveCommand == 0)) 
		{
			xjoymoveSmoothed = 0;
		}

		
		//set position
		motionManual.xPosition = xPositionCurrent;

		
		//-------- Z --------
		//limit yPositionCurrent changes
		if (zjoymoveSmoothed <  zjoymoveCommand)
		{
			zjoymoveSmoothed+=JOYSTICK_SMOOTHING;
		}
		if (zjoymoveSmoothed >  zjoymoveCommand)
		{
			zjoymoveSmoothed-=JOYSTICK_SMOOTHING;
		}
		if (zjoymoveCommand == 0)
		{
			if ((zjoymoveSmoothed <= JOYSTICK_SMOOTHING) && (zjoymoveSmoothed >= -JOYSTICK_SMOOTHING))
			{
				zjoymoveSmoothed = 0;
			}
		}

		
		//boundary smoothing
		if (zjoymoveSmoothed > 0)
		{		
			if (zPositionCurrent > systemBoundary.zmax - BOUNDARY_SMOOTHING)
			{
				moveBoundary = zjoymoveSmoothed * ((systemBoundary.zmax - zPositionCurrent)/BOUNDARY_SMOOTHING);
				zPositionCurrent = zPositionCurrent + moveBoundary;
			}
			else
			{
				zPositionCurrent = zPositionCurrent + zjoymoveSmoothed;
			}
		}
		
		if (zjoymoveSmoothed <= 0)
		{
			if (zPositionCurrent < systemBoundary.zmin + BOUNDARY_SMOOTHING)
			{
				moveBoundary = zjoymoveSmoothed * ((systemBoundary.zmin - zPositionCurrent)/BOUNDARY_SMOOTHING) * -1.0;
				zPositionCurrent = zPositionCurrent + moveBoundary;
			}
			else
			{
				zPositionCurrent = zPositionCurrent + zjoymoveSmoothed;
			}
		}
		
		//double check to make sure position can't be outside boundary
		if (zPositionCurrent > systemBoundary.zmax)
		{
			zPositionCurrent = systemBoundary.zmax;
		}
		if (zPositionCurrent < systemBoundary.zmin)
		{
			zPositionCurrent = systemBoundary.zmin;
		}
		
		//reset zjoymovesmoothed if at (or near) boundary
		if ((zPositionCurrent >= systemBoundary.zmax - BOUNDARY_JOYRESET) && (zjoymoveCommand == 0)) 
		{
			zjoymoveSmoothed = 0;
		}
		if ((zPositionCurrent <= systemBoundary.zmin + BOUNDARY_JOYRESET) && (zjoymoveCommand == 0)) 
		{
			zjoymoveSmoothed = 0;
		}

		
		//set position
		motionManual.zPosition = zPositionCurrent;
		
		
		//-------- Y --------
		//limit yPositionCurrent changes
		if (yjoymoveSmoothed <  yjoymoveCommand)
		{
			yjoymoveSmoothed+=JOYSTICK_SMOOTHING;
		}
		if (yjoymoveSmoothed >  yjoymoveCommand)
		{
			yjoymoveSmoothed-=JOYSTICK_SMOOTHING;
		}
		if (yjoymoveCommand == 0)
		{
			if ((yjoymoveSmoothed <= JOYSTICK_SMOOTHING) && (yjoymoveSmoothed >= -JOYSTICK_SMOOTHING))
			{
				yjoymoveSmoothed = 0;
			}
		}

		//check for floor on/off
		if (systemBoundary.floor == OFF)
		{
			systemBoundary.ymin = 0.0;
		}
		else
		{
			systemBoundary.ymin = systemBoundary.yminSaved; 
		}

		//boundary smoothing
		if (yjoymoveSmoothed > 0)
		{		
			if (yPositionCurrent > systemBoundary.ymax - BOUNDARY_SMOOTHING)
			{
				moveBoundary = yjoymoveSmoothed * ((systemBoundary.ymax - yPositionCurrent)/BOUNDARY_SMOOTHING);
				yPositionCurrent = yPositionCurrent + moveBoundary;
			}
			else
			{
				yPositionCurrent = yPositionCurrent + yjoymoveSmoothed;
			}
		}
		
		if (yjoymoveSmoothed <= 0)
		{
			if (yPositionCurrent < systemBoundary.ymin + BOUNDARY_SMOOTHING)
			{
				moveBoundary = yjoymoveSmoothed * ((systemBoundary.ymin - yPositionCurrent)/BOUNDARY_SMOOTHING) * -1.0;
				yPositionCurrent = yPositionCurrent + moveBoundary;
			}
			else
			{
				yPositionCurrent = yPositionCurrent + yjoymoveSmoothed;
			}
		}
		
		//double check to make sure position can't be outside boundary
		if (yPositionCurrent > systemBoundary.ymax)
		{
			yPositionCurrent = systemBoundary.ymax;
		}
		if (yPositionCurrent < systemBoundary.ymin)
		{
			yPositionCurrent = systemBoundary.ymin;
		}

		//reset xjoymovesmoothed if at (or near) boundary
		if ((yPositionCurrent >= systemBoundary.ymax - BOUNDARY_JOYRESET) && (yjoymoveCommand == 0)) 
		{
			yjoymoveSmoothed = 0;
		}
		if ((yPositionCurrent <= systemBoundary.ymin + BOUNDARY_JOYRESET) && (yjoymoveCommand == 0)) 
		{
			yjoymoveSmoothed = 0;
		}

		
		//set position
		motionManual.yPosition = yPositionCurrent;
		
		motionCommand.xPosition = motionManual.xPosition;
		motionCommand.yPosition = motionManual.yPosition;
		motionCommand.zPosition = motionManual.zPosition;
		
		sprintf(strUSBBuffer, "%s", writeBuffer);
		
		} //end manual mode

		
		//====================== AUTO MODE (PLAYBACK) =========================
		
		
		if ((systemMode == MODE_AUTO) && (systemStatus.Master == OK))
		{
			if (playbackMode == PLAYBACK_MODE_PLAY)
			{
				playbackSpeedRate = 100/playbackSpeedPercentage;
				
				playbackSpeedFrameCounter--;
				if (playbackSpeedFrameCounter <= 0)
				{
					playbackSpeedFrameCounter = playbackSpeedRate;
					mdcount++;
				}

				playbackDecelDiff = motionData[mdcount+1].xPosition - motionData[mdcount].xPosition;
				motionCommand.xPosition = motionCommand.xPosition + (playbackDecelDiff/playbackSpeedRate);
				playbackDecelDiff = motionData[mdcount+1].yPosition - motionData[mdcount].yPosition;
				motionCommand.yPosition = motionCommand.yPosition + (playbackDecelDiff/playbackSpeedRate);
				playbackDecelDiff = motionData[mdcount+1].zPosition - motionData[mdcount].zPosition;
				motionCommand.zPosition = motionCommand.zPosition + (playbackDecelDiff/playbackSpeedRate);

				//motionCommand.xPosition = motionData[mdcount].xPosition;
				//motionCommand.yPosition = motionData[mdcount].yPosition;
				//motionCommand.zPosition = motionData[mdcount].zPosition;
			}


			if (playbackMode == PLAYBACK_MODE_DECEL)
			{
				if (playbackDecelCount == 0)
				{
					mdcount++;
					playbackDecelValue++;
					playbackDecelCount = playbackDecelValue;
				}
				if (playbackDecelValue >= 15)
				{
					playbackMode = PLAYBACK_MODE_STOP;
				}

				playbackDecelCount--;
				
				playbackDecelDiff = motionData[mdcount+1].xPosition - motionData[mdcount].xPosition;
				motionCommand.xPosition = motionCommand.xPosition + (playbackDecelDiff/playbackDecelValue);
				playbackDecelDiff = motionData[mdcount+1].yPosition - motionData[mdcount].yPosition;
				motionCommand.yPosition = motionCommand.yPosition + (playbackDecelDiff/playbackDecelValue);
				playbackDecelDiff = motionData[mdcount+1].zPosition - motionData[mdcount].zPosition;
				motionCommand.zPosition = motionCommand.zPosition + (playbackDecelDiff/playbackDecelValue);
			}

			//to keep manual position in sync
			motionManual.xPosition = motionCommand.xPosition;
			motionManual.yPosition = motionCommand.yPosition;
			motionManual.zPosition = motionCommand.zPosition;
			xPositionCurrent = motionCommand.xPosition;
			yPositionCurrent = motionCommand.yPosition;
			zPositionCurrent = motionCommand.zPosition;

			if (mdcount >= motionDataFrames - 2) //make this 2 to keep from jumping at end of motion
			{
				playbackMode = PLAYBACK_MODE_STOP;
				systemAtPlayBackEnd = TRUE;
			}
			else
			{
				systemAtPlayBackEnd = FALSE;
			}
		} //end auto mode
		

		//=====================send pan data (auto or manual mode)====================
		// dont send data if in locked mode
		if (systemMode == MODE_AUTO)
		{
			sprintf(writeBuffer,"%06.2f\n",(motionData[mdcount].panMultiTurn)); //auto pan data
			bytesWritten = write(usbPortFD,writeBuffer,strlen(writeBuffer));
		}
		if (systemMode == MODE_MANUAL)
		{
			panManualMultiTurnValue = panManualMultiTurnValue + (pjoymoveCommand/0.5); //divisor sets pan speed
			panManualMultiTurnValue = limitf(panManualMultiTurnValue, -360.0, 360.0);
			sprintf(writeBuffer,"%06.2f\n",(panManualMultiTurnValue)); //Pan CCW
			bytesWritten = write(usbPortFD,writeBuffer,strlen(writeBuffer));
		}


		//=====================calculate speed data for winches (auto or manual mode)====================
		for (i = 0; i < 4; i++)
		{
			//testing only - winchData[i].PositionError = (float_t)(winchData[i].speedActual - winchData[i].speedCommand)/10.0; 

			//calculate new line lengths
			term1 = pulleyLocation[i].xLocation - motionCommand.xPosition;
			term2 = pulleyLocation[i].yLocation - motionCommand.yPosition;
			term3 = pulleyLocation[i].zLocation - motionCommand.zPosition;

			winchData[i].LineLength = sqrtf((term1*term1)+(term2*term2)+(term3*term3));
			//winchData[i].LineLengthFromZero = winchData[i].LineLength - winchData[i].LineLengthAtZero;
						
			if (winchData[i].LineLengthPrev == 0) //eliminate issue with first calc
			{
				winchData[i].LineLengthPrev = winchData[i].LineLength;
				winchData[i].LLoffset = winchData[i].LineLength;
				winchData[i].PAoffset = winchData[i].PositionActual;
			}

			winchData[i].LineLengthChange = winchData[i].LineLength - winchData[i].LineLengthPrev;
			winchData[i].LineLengthPrev = winchData[i].LineLength;
			
			//validate line length change
			if (abs(winchData[i].LineLengthChange) > 60.0) //18.0 m/s max rate
			{
				systemStatus.PositionChange = ERROR;
			}

			//calculate velocity for position change
			if (setCurrentPositionLockout == FALSE)
			{
//**************************************************************************************************
				winchData[i].speedCommand = winchData[i].LineLengthChange * CM_PER_FRAME_TO_RPM;
//**************************************************************************************************		
				if (systemMode == MODE_AUTO)
				{
				//determine line length from speed command
				actualLLC[i] = winchData[i].speedCommand / CM_PER_FRAME_TO_RPM;
				diffLLC[i] = actualLLC[i] - winchData[i].LineLengthChange;
				cumultiveDiffLLC[i] = cumultiveDiffLLC[i] + diffLLC[i];
				if (fabs(cumultiveDiffLLC[i]) > maxDiffLLC[i])
					maxDiffLLC[i] = fabs(cumultiveDiffLLC[i]);
				//printf ("Frame %05ld  Spd Diff: %04d -- diffLLC: %3.2f maxDiffLLC:%3.2f\n", frameCount,  
				//	winchData[0].speedCommandPrev - winchData[0].speedCommand, diffLLC, maxDiffLLC);	
				}
				
				winchData[i].speedCommandPrev = winchData[i].speedCommand;
			}
			else
			{
				winchData[i].speedCommand = 0;
			}
			
			
			//--- check for winch jog command
			//--- only allow jog if in manual mode, servos are on and not currently moving
			if  ((systemMode == MODE_MANUAL) && (systemStatus.Servo == OK) && (winchData[i].speedCommand == 0.0)) //if no speed command then can jog
			{
				if (winchData[i].JogStatus == JOG_IN)
				{
					winchData[i].speedCommand = jogSpeed * -1;
				}
				if (winchData[i].JogStatus == JOG_OUT)
				{
					winchData[i].speedCommand = jogSpeed;
				}
			}
			
			
			
		} //end winch for loop
		
		
		if (setCurrentPositionLockout == TRUE)
		{
			setCurrentPositionLockout = FALSE;
		}
		


		//determine if system is currently moving 
		if ((winchData[0].speedCommand != 0.0) ||
			(winchData[1].speedCommand != 0.0) ||
			(winchData[2].speedCommand != 0.0) ||
			(winchData[3].speedCommand != 0.0))
		{
			systemStatus.Moving = TRUE;
		}
		else
		{
			systemStatus.Moving = FALSE;
		}
		

		//=====================send speed data to winches (auto or manual mode)====================
		for (i=0;i<4;i++)
		{
			if ((systemStatus.Comms == OK) && (simulationTest == FALSE))
			{
				speedval = winchData[i].speedCommand;
			}
			else
			{
				speedval = 0;
			}

			if (sendAlarmReset == TRUE)
			{
				sendAlarmResetCounter++;
				if (sendAlarmResetCounter > 7)
				{
					sendAlarmReset = FALSE;
				}
				sprintf(writeBuffer,":010604070020CE\r\n");  //send alarm reset command
				bytesWritten = write(serialPortFD[i],writeBuffer,strlen(writeBuffer));
				sprintf(strWriteBuffer, "%s", writeBuffer);
			}
			
			else if (sendServoOnOff == TRUE)
			{
				sendServoOnOffCounter++;
				if (sendServoOnOffCounter > 30)
				{
					sendServoOnOff = FALSE;
				}

				//if servo command is on, then
				// - in first 6 frames send brake output enabled (controlled by servo amp)
				// - in second 6 frames send servo on
				if ((setServoOnOff == ON) && ((sendServoOnOffCounter >= 1) && (sendServoOnOffCounter < 7)))
				{
					sprintf(writeBuffer,":01060212006C79\r\n"); //send brake output enabled command
				}
				if ((setServoOnOff == ON) && ((sendServoOnOffCounter >= 7) && (sendServoOnOffCounter < 13)))
				{
					sprintf(writeBuffer,":010604070010DE\r\n"); //send servo on command
				}

				//if servo command is off, then
				// - in first 6 frames send brake enabled off
					//sprintf(writeBuffer,":010602120000E5\r\n"); //send brake output disabled command
				// - in second 12 frames (400ms) wait
				// - in third 6 frames send servo off
				// - in forth 6 frame send brake output enabled (controlled by servo amp)
				if ((setServoOnOff == OFF) && ((sendServoOnOffCounter >= 1) && (sendServoOnOffCounter < 7)))
				{
					sprintf(writeBuffer,":010602120000E5\r\n"); //send brake output disabled command
				}
				
				//do nothing when sendServoOnOffCounter >= 7 and < 19 -- i.e. wait 400 ms
				
				if ((setServoOnOff == OFF) && ((sendServoOnOffCounter >= 19) && (sendServoOnOffCounter < 25)))
				{
					sprintf(writeBuffer,":010604070000EE\r\n"); //send servo off command
				}
				if ((setServoOnOff == OFF) && ((sendServoOnOffCounter >= 25) && (sendServoOnOffCounter < 31)))
				{
					sprintf(writeBuffer,":01060212006C79\r\n"); //send brake output enabled command
				}

				bytesWritten = write(serialPortFD[i],writeBuffer,strlen(writeBuffer));
				sprintf(strWriteBuffer, "%s", writeBuffer);
			}
			
			else //no alarm/servo commands to send, send speed commands
			{
				int8_t speedMSB;
				int8_t speedLSB;
				speedval = speedval * -1; //reverse motor direction
				speedMSB = (int8_t)(speedval >> 8);
				speedLSB = (int8_t)(speedval);
				uint16_t lrcval;
				lrcval = 65536 - (speedMSB + speedLSB + 17); //17 = 01+06+01+09
				sprintf(writeBuffer,":01060109%04X%02X\r\n", (uint16_t)speedval, (uint8_t)lrcval);
				bytesWritten = write(serialPortFD[i],writeBuffer,strlen(writeBuffer));
				sprintf(strWriteBuffer, "%s", writeBuffer);
			}
		} //end winches for loop (sending speed to each winch)
	} //end else (toggle every other iteration)

	} //end if frame count is greater than 60

	// --- calculate average loop time ---
	if (avgTimeCount2 >= AVGCOUNT2)
			avgTimeCount2 = 0;
	avgTimeValue2[avgTimeCount2] = threadDelta2/1000.0;
	avgTimeCount2++;
	int i;
	avgTime2 = 0.0;
	for (i=0;i<AVGCOUNT2;i++)
	{
		avgTime2 = avgTime2 + avgTimeValue2[i];
	}
	avgTime2 = avgTime2/AVGCOUNT2;


	//calculate loop time
	gettimeofday(&threadEnd2, NULL);
	if (threadStart2.tv_usec > threadEnd2.tv_usec) //take care of crossing into new second
		threadEnd2.tv_usec += 1000000;
	threadDelta2 = threadEnd2.tv_usec - threadStart2.tv_usec;
    gettimeofday(&threadStart2, NULL);  //should be last instruction

	} // while(1) loop
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Library functions


//limitf - this function returns the function value limited by min/max
float limitf(float value, float min, float max)
{
	if (value < min)
	{
		return min;
	}
	else if (value > max)
	{
		return max;
	}
	else
	{
		return value;
	}
}

int limit_int(int value, int min, int max)
{
	if (value < min)
	{
		return min;
	}
	else if (value > max)
	{
		return max;
	}
	else
	{
		return value;
	}
}


void updateTimeCode (void)
{
	tc_frames++;
	if (tc_frames >= 60)
	{
		tc_frames = 0;
		tc_sec++;
		if (tc_sec >= 60)
		{
			tc_sec = 0;
			tc_min++;
			if (tc_min >= 60)
			{
				tc_min = 0;
				tc_hr++;
			}
		}
	}
}


int setupSerialPort (int winch, char* portName)
{
	int port_fd;
	port_fd = open(portName,(O_RDWR | O_NOCTTY | O_NONBLOCK));
	if (port_fd == -1)
		printf("\nError! in opening winch %d serial port %s", winch, portName);
	else
		{
		printf("\nSuccessfully opened winch %d serial port %s [fd=%d]",winch, portName, port_fd);
		tcgetattr(port_fd,&SerialPortSettings);
		cfsetispeed(&SerialPortSettings,B57600);
		cfsetospeed(&SerialPortSettings,B57600);
		//cfsetispeed(&SerialPortSettings,B115200);
		//cfsetospeed(&SerialPortSettings,B115200);
		SerialPortSettings.c_cflag &= ~PARENB; //set no parity
		//SerialPortSettings.c_cflag &= ~CSTOPB; //set 1 stop bit
		SerialPortSettings.c_cflag |= CSTOPB; //set 2 stop bit
		SerialPortSettings.c_cflag &= ~CSIZE; //clear the bit mask
		SerialPortSettings.c_cflag |= CS8; //set data bits to 8
		SerialPortSettings.c_cflag &= ~CRTSCTS; //no flow control
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; //turn on receiver
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); //no sw flow Control
		SerialPortSettings.c_oflag &= ~ONLCR; //do not map \n to \r\n ***important!
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		tcsetattr(port_fd, TCSANOW, &SerialPortSettings);
		}
	return port_fd;
}

int setupUSBSerialPort (char* portName)
{
	int port_fd;
	port_fd = open(portName,(O_RDWR | O_NOCTTY | O_NONBLOCK));
	if (port_fd == -1)
		printf("\nError! in opening PTR serial port %s", portName);
	else
		{
		printf("\nSuccessfully opened PTR serial %s [fd=%d]",portName, port_fd);
		tcgetattr(port_fd,&SerialPortSettings);
		cfsetispeed(&SerialPortSettings,B57600);
		cfsetospeed(&SerialPortSettings,B57600);
		SerialPortSettings.c_cflag &= ~PARENB; //set no parity
		SerialPortSettings.c_cflag &= ~CSTOPB; //set 1 stop bit
		//SerialPortSettings.c_cflag |= CSTOPB; //set 2 stop bit
		SerialPortSettings.c_cflag &= ~CSIZE; //clear the bit mask
		SerialPortSettings.c_cflag |= CS8; //set data bits to 8
		SerialPortSettings.c_cflag &= ~CRTSCTS; //no flow control
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; //turn on receiver
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); //no sw flow Control
		SerialPortSettings.c_oflag &= ~ONLCR; //do not map \n to \r\n ***important!
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		tcsetattr(port_fd, TCSANOW, &SerialPortSettings);
		}
	return port_fd;
}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


gboolean timeoutFunction (gpointer data)
{
	//-------------------------------------------------------------------------
	//  Manual Winch Control for direct drive speed tests
	//-------------------------------------------------------------------------
	//int winchNumber = WINCH_03;
	//int winchRPM = 1000;
	//int manualControl = OFF;
	//if (manualControl != OFF)
	//{
	//	winchData[winchNumber].speedCommand = joyValue1 * -1.0;
	//	if (winchData[winchNumber].speedCommand > winchRPM)
	//		winchData[winchNumber].speedCommand = winchRPM;
	//	if (winchData[winchNumber].speedCommand < -winchRPM)
	//		winchData[winchNumber].speedCommand = -winchRPM;
	//}
	//-------------------------------------------------------------------------

	
	int i;
	for (i = 0; i < 4; i++)
	{
		LLcm[i] = winchData[i].LineLength - winchData[i].LLoffset;
		PAcm[i] = (winchData[i].PositionActual - winchData[i].PAoffset) * CM_PER_MOTOR_REV * -1;
		DFcm[i] = LLcm[i] - PAcm[i];
		winchData[i].PositionError = DFcm[i];
	}

	sprintf(strMisc, "LL:%6.2fcm  PA:%6.2fcm  DF:%6.3fcm", LLcm[0], PAcm[0], DFcm[0]);
	gtk_label_set_text(GTK_LABEL(g_lblPosition1), strMisc);
	sprintf(strMisc, "LL:%6.2fcm  PA:%6.2fcm  DF:%6.3fcm", LLcm[1], PAcm[1], DFcm[1]);
	gtk_label_set_text(GTK_LABEL(g_lblPosition2), strMisc);
	sprintf(strMisc, "LL:%6.2fcm  PA:%6.2fcm  DF:%6.3fcm", LLcm[2], PAcm[2], DFcm[2]);
	gtk_label_set_text(GTK_LABEL(g_lblPosition3), strMisc);
	sprintf(strMisc, "LL:%6.2fcm  PA:%6.2fcm  DF:%6.3fcm", LLcm[3], PAcm[3], DFcm[3]);
	gtk_label_set_text(GTK_LABEL(g_lblPosition4), strMisc);


	//sprintf(strMisc, "POS DIFF: %06.2f  %06.2f  %06.2f  %06.2f", maxDiffLLC[0], maxDiffLLC[1], maxDiffLLC[2], maxDiffLLC[3]);
	sprintf(strMisc, "ENCDR: %06.4f  %06.4f  %06.4f  %06.4f", winchData[0].PositionActual, winchData[1].PositionActual, winchData[2].PositionActual, winchData[3].PositionActual);
	gtk_label_set_text(GTK_LABEL(g_lblDebug4), strMisc);

	//sprintf(strMisc, "XCMD: %06.2f  XPOS: %06.2f  DIFF: %06.2f", motionCommand.xPosition, motionData[mdcount].xPosition, motionCommand.xPosition - motionData[mdcount].xPosition);
	sprintf(strMisc, "XMC: %06.2f  YMC: %06.2f  ZMC: %06.2f", xjoymoveCommand, yjoymoveCommand, zjoymoveCommand);	
	gtk_label_set_text(GTK_LABEL(g_lblDebug3), strMisc);

	//sprintf(strMisc, "%06.2f  %06.2f  %06.2f  %06.2f", cumultiveDiffLLC[0], cumultiveDiffLLC[1], cumultiveDiffLLC[2], cumultiveDiffLLC[3]);
	sprintf(strMisc, "CMD: %d  ACT: %d  ERR: %03.2f", winchData[0].speedCommand, winchData[0].speedActual, winchData[0].PositionError);
	gtk_label_set_text(GTK_LABEL(g_lblDebug1), strMisc);


	//sprintf(strMisc, "JOYSTICK PAN: %6.3f", pjoymoveCommand);
	sprintf(strMisc, "MANUAL PAN: %06.2f", panManualMultiTurnValue);
	gtk_label_set_text(GTK_LABEL(g_lbl_joyvalue), strMisc);

	sprintf(strMisc, "MOTION PAN: %06.2f", motionData[mdcount].panMultiTurn);
	gtk_label_set_text(GTK_LABEL(g_lblDebug2), strMisc);

	//gtk_label_set_text(GTK_LABEL(g_lblJogTest), "Not Jogging");




	chartTimeCounter--;
	if (chartTimeCounter < 0)
	{
		chartTimeCounter = 99;
		chartTimeSeconds+=5; //increment 5 seconds for every 100 ticks (50ms ticks)
	}

	//update labels
	gtk_label_set_text(GTK_LABEL(g_lbl_serdata), strWriteBuffer);
	gtk_label_set_text(GTK_LABEL(g_lbl_serdatarcv0), strReceivedDisplayBuffer[0]);
	gtk_label_set_text(GTK_LABEL(g_lbl_serdatarcv1), strReceivedDisplayBuffer[1]);
	gtk_label_set_text(GTK_LABEL(g_lbl_serdatarcv2), strReceivedDisplayBuffer[2]);
	gtk_label_set_text(GTK_LABEL(g_lbl_serdatarcv3), strReceivedDisplayBuffer[3]);


	if (systemBoundary.floor == OFF)
	{
		gtk_button_set_label(g_btnFloor, "FLOOR OFF");
	}
	else
	{
		gtk_button_set_label(g_btnFloor, "FLOOR ON");
	}



	if (systemMode == MODE_LOCKED)
	{
		gtk_label_set_text(GTK_LABEL(g_lblModeStatus1), "LOCKED MODE");
		gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), "");
	}
	if (systemMode == MODE_MANUAL)
	{
		gtk_label_set_text(GTK_LABEL(g_lblModeStatus1), "MANUAL MODE");
		gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), "");
	}
	if (systemMode == MODE_AUTO)
	{
		gtk_label_set_text(GTK_LABEL(g_lblModeStatus1), "AUTO MODE");
		if (playbackMode == PLAYBACK_MODE_PLAY)
		{
			sprintf(strMisc, "PLAYING %s [%04d]", motionData[mdcount].timecode, motionData[mdcount].frameNumber);
			gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), strMisc);
		}
		if (playbackMode == PLAYBACK_MODE_STOP)
		{
			sprintf(strMisc, "STOPPED %s [%04d]", motionData[mdcount].timecode, motionData[mdcount].frameNumber);
			gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), strMisc);
		}
		if (playbackMode == PLAYBACK_MODE_DECEL)
		{
			sprintf(strMisc, "DECEL %s [%04d]", motionData[mdcount].timecode, motionData[mdcount].frameNumber);
			gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), strMisc);
		}
		if (playbackMode == PLAYBACK_MODE_GOTOSTART) //priority message over playback modes
		{
			gtk_label_set_text(GTK_LABEL(g_lblModeStatus2), "MOVING TO START");
		}
	}
	
	if ((playbackMode == PLAYBACK_MODE_STOP) || (playbackMode == PLAYBACK_MODE_GOTOSTART))
	{
		if (systemAtPlayPosition == TRUE)
		{
			gtk_label_set_text(GTK_LABEL(g_lblPlayBackStatus), "AT PLAY POSITION");
			if (systemAtPlayBackEnd == TRUE)
			{
				gtk_label_set_text(GTK_LABEL(g_lblPlayBackStatus), "AT PLAYBACK END");
			}
		}
		else
		{
			gtk_label_set_text(GTK_LABEL(g_lblPlayBackStatus), "NOT AT PLAY POSITION");
		}
	}
	else
	{
		sprintf(strMisc, "PLAYING BACK AT %d%%", playbackSpeedPercentage);
		gtk_label_set_text(GTK_LABEL(g_lblPlayBackStatus), strMisc);		
	}

    //update thread chart data array
	for (int j=0;j<600; j++) //shift data points left
	{
		threadChartData[j] = threadChartData[j+1];
	}
	//add new winch speed to last data point
//	threadChartData[600] = (int)(threadDelta2/5.0) - (16666/5);
	threadChartData[600] = (int)(threadDelta2/5.0) - (33332/5);

    //update winch chart data array
    for (int i = 0; i < 4; i++)
    {
		for (int j=0;j<485; j++) //shift data points left
		{
			winchChartSpeedCommandData[i][j] = winchChartSpeedCommandData[i][j+1]; //speed
			winchChartSpeedData[i][j] = winchChartSpeedData[i][j+1]; //speed
			winchChartLoadData[i][j] = winchChartLoadData[i][j+1]; //load
		}
		//add new winch speed to last data point
		winchChartSpeedCommandData[i][485] = winchData[i].speedCommand * -10; 
		winchChartSpeedData[i][485] = winchData[i].speedActual * -10; 
		winchChartLoadData[i][485] = winchData[i].motorLoad * -1;
	}

	//update the winch telemetry display
	gtk_widget_queue_draw (g_drawgauge1);
	gtk_widget_queue_draw (g_drawgauge2);
	gtk_widget_queue_draw (g_drawgauge3);
	gtk_widget_queue_draw (g_drawgauge4);

	//update pan,tilt,roll head telemetry display
	gtk_widget_queue_draw (g_drawPTR);

	//update path display
	gtk_widget_queue_draw (g_drawFloor);
	gtk_widget_queue_draw (g_drawElevation);

	gtk_widget_queue_draw (g_drawXYZ);
	gtk_widget_queue_draw (g_drawarea2);
	gtk_widget_queue_draw (g_drawarea);
	gtk_widget_queue_draw (g_drawStatus);
	gtk_widget_queue_draw (g_drawServoStatus);
	gtk_widget_queue_draw (g_drawAlarmStatus);
	
	gtk_widget_queue_draw (g_drawSpeedChart);
	

	//flash toggle - used to flash display items
	flashToggleCount++;
	if (flashToggleCount >= FLASH_TOGGLE_RATE)
	{
		flashToggleCount = 0;
		if (flashToggle == TRUE)
		{
			flashToggle = FALSE;
		}
		else
		{
			flashToggle = TRUE;
		}
	}

	//gdk_threads_add_timeout (50, timeoutFunction, NULL); //add new timeout with new time
	return TRUE;  //TRUE = auto restarts, FALSE = cancel restart
}

//=== JOG FUNCTIONS

void on_btnJogInWinch1_pressed (void)
{
		gtk_label_set_text(GTK_LABEL(g_lblJogWinch1), "Jogging In");
		winchData[0].JogStatus = JOG_IN;	
}
void on_btnJogInWinch1_released (void)
{
		gtk_label_set_text(GTK_LABEL(g_lblJogWinch1), "Not Jogging");
		winchData[0].JogStatus = JOG_OFF;	
}

void on_btnJogOutWinch1_pressed (void)
{
		gtk_label_set_text(GTK_LABEL(g_lblJogWinch1), "Jogging Out");
		winchData[0].JogStatus = JOG_OUT;	
}
void on_btnJogOutWinch1_released (void)
{
		gtk_label_set_text(GTK_LABEL(g_lblJogWinch1), "Not Jogging");
		winchData[0].JogStatus = JOG_OFF;	
}


void on_btnJogInWinch2_pressed (void)
{
		winchData[1].JogStatus = JOG_IN;	
}
void on_btnJogInWinch2_released (void)
{
		winchData[1].JogStatus = JOG_OFF;	
}
void on_btnJogOutWinch2_pressed (void)
{
		winchData[1].JogStatus = JOG_OUT;	
}
void on_btnJogOutWinch2_released (void)
{
		winchData[1].JogStatus = JOG_OFF;	
}


void on_btnJogInWinch3_pressed (void)
{
		winchData[2].JogStatus = JOG_IN;	
}
void on_btnJogInWinch3_released (void)
{
		winchData[2].JogStatus = JOG_OFF;	
}
void on_btnJogOutWinch3_pressed (void)
{
		winchData[2].JogStatus = JOG_OUT;	
}
void on_btnJogOutWinch3_released (void)
{
		winchData[2].JogStatus = JOG_OFF;	
}


void on_btnJogInWinch4_pressed (void)
{
		winchData[3].JogStatus = JOG_IN;	
}
void on_btnJogInWinch4_released (void)
{
		winchData[3].JogStatus = JOG_OFF;	
}
void on_btnJogOutWinch4_pressed (void)
{
		winchData[3].JogStatus = JOG_OUT;	
}
void on_btnJogOutWinch4_released (void)
{
		winchData[3].JogStatus = JOG_OFF;	
}

void on_cbJogSpeed_changed() 
{
	switch (gtk_combo_box_get_active(g_cbJogSpeed))
	{
		case 0:
			jogSpeed = 1;
			break;
		case 1:
			jogSpeed = 5;
			break;
		case 2:
			jogSpeed = 10;
			break;
		case 3:
			jogSpeed = 25;
			break;
		case 4:
			jogSpeed = 50;
			break;
		case 5:
			jogSpeed = 100;
			break;
		case 6:
			jogSpeed = 250;
			break;
		case 7:
			jogSpeed = 500;
			break;
	}
}


void on_filechooserbutton1_file_set (void)
{
	char *filename;
	filename = gtk_file_chooser_get_filename (g_filechooser);
	gtk_list_store_clear(g_motionDataStore);
	readMotionFile(filename);
	appendMotionData();
	gtk_text_buffer_set_text (g_txtBuffer,"File Requires Validation", -1);
	gtk_text_view_set_buffer (g_txtStats,g_txtBuffer);
	motionDataFileValidated = FALSE;
	//auto validate
	on_btnValidate_clicked();


}


void readMotionFile (char *filename)
{
	char buffer[1024] ;
	char *record,*line;
	int i=0,j=0;
	char tempValue[10][20];
	FILE *fstream = fopen(filename,"r");
	if(fstream == NULL)
	{
      printf("\n file opening failed ");
	}
	else
	{
		//reinit key values (current frame, start frame, pan revs)
		startFrame = 1;
		gtk_entry_set_text(g_shiftFrame, "1");
		mdcount = 0;
		pan360Rev = 0; 
		
		while((line=fgets(buffer,sizeof(buffer),fstream))!=NULL)
		{
			//if (line[0] != 'F') //skips first line, expected to be header line with "Frame" as first characters
				j=0;
				record = strtok(line,",");
				while(record != NULL)
				{
					strcpy(tempValue[j++], record);
					record = strtok(NULL,",");
				}
				if (atoi(tempValue[0]) != 0)
				{
					motionData[i].frameNumber = atoi(tempValue[0]);
					strcpy(motionData[i].timecode, tempValue[1]);
					motionData[i].xPosition = atof(tempValue[2]);
					motionData[i].yPosition = atof(tempValue[3]) + PTR_Y_OFFSET;
					motionData[i].zPosition = atof(tempValue[4]);
					
					//calculate multiturn pan info
					
					panInputValue = atof(tempValue[5]);
					if (panInputValue < 0)
					{
						pan360Value = panInputValue + 360.0; //convert +/- 180 to 0 to 360
					}
					else
					{
						pan360Value = panInputValue;
					}

					if (i == 0) //set previous to current value first time through
					{
						pan360Previous = pan360Value;
					}
					
					
					//calculate rotations (moving CW or CCW)
					if ((pan360Previous > 270.0) && (pan360Value < 90.0)) //moving Q4 to Q1
					{
						pan360Rev++;
					}
					if ((pan360Previous < 90.0) && (pan360Value > 270.0)) //moving Q1 to Q4
					{
						pan360Rev--;
					}
					pan360Previous = pan360Value;

					panMultiTurnValue = pan360Value + (pan360Rev * 360);
					motionData[i].panMultiTurn = panMultiTurnValue;

					motionData[i].panRotation = panInputValue;
					motionData[i].tiltRotation = atof(tempValue[6]);
					motionData[i].rollRotation = atof(tempValue[7]);
					i++;
				}
		}
		motionData[i].frameNumber = -1; //identifies last frame
		motionDataFrames = i;
		chartZoom = (motionDataFrames/545) + 1; //set intial chart zoom to include all
	}
}

void appendMotionData (void)
{
	int i=0;
	char strVal[15][30];
	while (motionData[i].frameNumber != -1)
	{
		//use string values to be able to format float value
		sprintf(strVal[2],"%3.2f", motionData[i].xPosition);
		sprintf(strVal[3],"%3.2f", motionData[i].yPosition);
		sprintf(strVal[4],"%3.2f", motionData[i].zPosition);
		sprintf(strVal[5],"%3.2f", motionData[i].panRotation);
		sprintf(strVal[6],"%3.2f", motionData[i].tiltRotation);
		sprintf(strVal[7],"%3.2f", motionData[i].rollRotation);
		
			gtk_list_store_append(g_motionDataStore, &g_treeIter);
			gtk_list_store_set (g_motionDataStore, &g_treeIter,
				0,motionData[i].frameNumber,
				1,motionData[i].timecode,
				2,strVal[2],
				3,strVal[3],
				4,strVal[4],
				5,strVal[5],
				6,strVal[6],
				7,strVal[7],
				-1);
		i++;
	}
}

void on_btnValidate_clicked (void)
{
	int i=0;
	char strStat[50];
	float xMin, xMax, xEnvelope;
	float yMin, yMax, yEnvelope;
	float zMin, zMax, zEnvelope;
	xMin = 1000.0;
	xMax = -1000.0;
	yMin = 1000.0;
	yMax = -1000.0;
	zMin = 1000.0;
	zMax = -1000.0;
	float xFrameDist, xSpeed, xMaxSpeed = 0.00;
	float yFrameDist, ySpeed, yMaxSpeed = 0.00;
	float zFrameDist, zSpeed, zMaxSpeed = 0.00;
	int xMSframe, yMSframe, zMSframe;

	pathWithinBoundary = FALSE;

	//get x,y,z envelope
	for (i=0;i<motionDataFrames;i++)
	{
		if (motionData[i].xPosition < xMin)
			xMin = motionData[i].xPosition;
		if (motionData[i].xPosition > xMax)
			xMax = motionData[i].xPosition;

		if (motionData[i].yPosition < yMin)
			yMin = motionData[i].yPosition;
		if (motionData[i].yPosition > yMax)
			yMax = motionData[i].yPosition;

		if (motionData[i].zPosition < zMin)
			zMin = motionData[i].zPosition;
		if (motionData[i].zPosition > zMax)
			zMax = motionData[i].zPosition;

		if (i<(motionDataFrames-1))
		{
			xFrameDist = abs(motionData[i+1].xPosition - motionData[i].xPosition);
			//xSpeed = (xFrameDist/100.0)/0.0333; //in m/s
			xSpeed = xFrameDist * 0.30; //cm/frame to m/s
			if (xSpeed > xMaxSpeed)
			{
				xMaxSpeed = xSpeed;
				xMSframe = i+1;
			}

			yFrameDist = abs(motionData[i+1].yPosition - motionData[i].yPosition);
			ySpeed = yFrameDist * 0.30; //in m/s
			if (ySpeed > yMaxSpeed)
			{
				yMaxSpeed = ySpeed;
				yMSframe = i+1;
			}

			zFrameDist = abs(motionData[i+1].zPosition - motionData[i].zPosition);
			zSpeed = zFrameDist * 0.30; //in m/s
			if (zSpeed > zMaxSpeed)
			{
				zMaxSpeed = zSpeed;
				zMSframe = i+1;
			}
		}

	}

	xEnvelope = xMax - xMin;
	yEnvelope = yMax - yMin;
	zEnvelope = zMax - zMin;
	
	//check to see if path is within boundary
	if ((xMax < systemBoundary.xmax) &&
		(xMin > systemBoundary.xmin) &&
		(yMax < systemBoundary.ymax) &&
		(yMin > systemBoundary.ymin) &&
		(zMax < systemBoundary.zmax) &&
		(zMin > systemBoundary.zmin))
		{
			pathWithinBoundary = TRUE;
		}

	sprintf(strStat,"Total Frames: %d (%3.2f seconds)\n",motionDataFrames, (float)motionDataFrames/30.0);
	gtk_text_buffer_set_text (g_txtBuffer,strStat, -1);
	//sprintf(strStat,"Total Time: %3.2f seconds\n",(float)motionDataFrames/30.0);
	//gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);

	sprintf(strStat,"X Envelope: %3.2f meters (%3.2f m to %3.2f m)\n",xEnvelope/100.0,xMin/100.0, xMax/100.0);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);
	sprintf(strStat,"Y Envelope: %3.2f meters (%3.2f m to %3.2f m)\n",yEnvelope/100.0,yMin/100.0, yMax/100.0);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);
	sprintf(strStat,"Z Envelope: %3.2f meters (%3.2f m to %3.2f m)\n",zEnvelope/100.0,zMin/100.0, zMax/100.0);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);

	sprintf(strStat,"X Max Speed: %3.2f m/s (at frame number %d)\n",xMaxSpeed, xMSframe);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);
	sprintf(strStat,"Y Max Speed: %3.2f m/s (at frame number %d)\n",yMaxSpeed, yMSframe);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);
	sprintf(strStat,"Z Max Speed: %3.2f m/s (at frame number %d)\n",zMaxSpeed, zMSframe);
	gtk_text_buffer_insert_at_cursor (g_txtBuffer,strStat, -1);
	
	//calculate and list winch speeds for each frame
	for (i=0;i<motionDataFrames;i++)
	{
		
	}
	

	gtk_text_view_set_buffer (g_txtStats,g_txtBuffer);

	motionDataFileValidated = TRUE;
}

void on_btnShiftData_clicked (void)
{
	const gchar *entryTextX;
	const gchar *entryTextY;
	const gchar *entryTextZ;

	if (motionDataFileValidated == TRUE)
	{
		gtk_list_store_clear(g_motionDataStore);

		entryTextX = gtk_entry_get_text(g_shiftX);
		entryTextY = gtk_entry_get_text(g_shiftY);
		entryTextZ = gtk_entry_get_text(g_shiftZ);

		for (int i = 0; i<motionDataFrames; i++)
		{
			motionData[i].xPosition = motionData[i].xPosition + atof(entryTextX);
			motionData[i].yPosition = motionData[i].yPosition + atof(entryTextY);
			motionData[i].zPosition = motionData[i].zPosition + atof(entryTextZ);
		}

		appendMotionData();

		//clear text entry boxes
		gtk_entry_set_text (g_shiftX, "");
		gtk_entry_set_text (g_shiftY, "");
		gtk_entry_set_text (g_shiftZ, "");
		
		//automatically revalidate file
		on_btnValidate_clicked();
	}
}

void on_btnShiftStart_clicked (void)
{
	const gchar *entryFrame;

	if (motionDataFileValidated == TRUE)
	{
		entryFrame = gtk_entry_get_text(g_shiftFrame);
		startFrame = atoi(entryFrame);
		startFrame = limit_int(startFrame, 1, motionDataFrames);
		sprintf(strMisc, "%d", startFrame);
		gtk_entry_set_text(g_shiftFrame, strMisc); //update text box with limited value
		mdcount = startFrame-1; //mdcount is 0 based, frame numbers start at 1
		pan360Rev = 0; //to keep from shifted data looking like more than one rev
	}
}

void on_treeview1_row_activated ()
{
		//startFrame = -----;
		//mdcount = startFrame;
}




gboolean on_joystick_change (GIOChannel *source, GIOCondition condition, gpointer data)
{
	struct js_event js;

		gsize len;
		GError *err = NULL;
		g_io_channel_read_chars(source, (void*)(&js), sizeof(struct js_event), &len, &err);

		if (js.type == JS_EVENT_AXIS)
		{
			if (js.number == 100) //100 is nothing
			{
				joyValue1 = js.value/32;

				if ((joyValue1 < 100) && (joyValue1 > -100))
				{
					joyValue1 = 0;
				}
				if (joyValue1 >= 100)
				{
					joyValue1 = joyValue1 - 100;
				}
				if (joyValue1 <= -100)
				{
					joyValue1 = joyValue1 + 100;
				}
				if (js.value >= 0)
				{
					gtk_progress_bar_set_fraction(g_pbar1, ((float)js.value/32767.0));
					gtk_progress_bar_set_fraction(g_pbar2, 0.0);
				}
				if (js.value < 0)
				{
					gtk_progress_bar_set_fraction(g_pbar2, ((float)js.value/32767.0)*-1.0);
					gtk_progress_bar_set_fraction(g_pbar1, 0.0);
				}
			}
			

			if (js.number == 0) //LH JOY RIGHT/LEFT
			{
				joyValue2 = js.value/32; //convert to appx 0 to 1024
				pjoymoveCommand = (float)joyValue2/-341.0; //sensitivity limit 0 to 3, reversed
				joytest1 = (float)joyValue2/-341.0; //sensitivity limit 0 to 3
			}

			if (js.number == 1) //LH JOY UP/DOWN
			{
				joyValue2 = js.value/32; //convert to appx 0 to 1024
				//yjoymoveCommand = (float)joyValue2/-341.0; //sensitivity limit 0 to 3, reversed
				//yjoymoveCommand = ((float)joyValue2/51.2); //sensitivity limit 0 to 20, not reversed
				yjoymoveCommand = ((float)joyValue2/-51.2); //sensitivity limit 0 to 20, reversed
				
				//add deadband
				if ((yjoymoveCommand < 0.2) && (yjoymoveCommand > -0.2))
				{
					yjoymoveCommand = 0;
				}
			}

			if (js.number == 3) //RH JOY RIGHT/LEFT
			{
				joyValue2 = js.value/32; //convert to appx 0 to 1024
				//xjoymoveCommand = (float)joyValue2/-102.4; //sensitivity limit 0 to 10, reversed
				//xjoymoveCommand = ((float)joyValue2/-25.6); //sensitivity limit 0 to 40, reversed
				xjoymoveCommand = ((float)joyValue2/25.6); //sensitivity limit 0 to 40, not reversed

				//add deadband
				if ((xjoymoveCommand < 0.2) && (xjoymoveCommand > -0.2))
				{
					xjoymoveCommand = 0;
				}
			}
			if (js.number == 4) //RH JOY UP/DOWN
			{
				joyValue2 = js.value/32; //convert to appx 0 to 1024
				//zjoymoveCommand = (float)joyValue2/102.4; //sensitivity limit 0 to 10 
				zjoymoveCommand = (float)joyValue2/25.6; //sensitivity limit 0 to 40
				//zjoymoveCommand = (float)joyValue2/-25.6; //sensitivity limit 0 to 40, reversed

				//add deadband
				if ((zjoymoveCommand < 0.2) && (zjoymoveCommand > -0.2))
				{
					zjoymoveCommand = 0;
				}
			}
		}
	return TRUE;
}

gboolean on_drawingarea2_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	struct tm* ptm;
	//guint width, height;

	cairo_set_line_width (cr, 2.0);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);

	//width = gtk_widget_get_allocated_width (widget);
	//height = gtk_widget_get_allocated_height (widget);

	//cairo_set_source_rgba (cr, 0.5, 0.5, 0.5, 1.0);
	//cairo_rectangle (cr, 1, 1, width-2, height-2);
	//cairo_stroke (cr);

	cairo_set_font_size(cr, 12);
	cairo_set_source_rgba (cr, YEL01);

	//display time of day
	gettimeofday(&realtime, NULL);
	ptm = localtime(&realtime.tv_sec);
	cairo_move_to(cr, 10, 15);
	strftime (strMisc, sizeof (strMisc), "%H:%M:%S", ptm);
	//do not use time of day display (to make room for version number)
	//cairo_show_text(cr, strMisc); 

	//cairo_move_to(cr, 10, 20);
	//sprintf(strMisc, "%03d:%02d:%02d.%02d",tc_hr,tc_min,tc_sec,tc_frames);
	//cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 10, 25);
	sprintf(strMisc, "%5.3f mS", threadDelta2/1000.0);
	cairo_show_text(cr, strMisc);
	
	cairo_move_to(cr, 10, 40);
	sprintf(strMisc, "%5.2f mS", avgTime2);
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, 10, 60);
	//sprintf(strMisc, "Version: %5.2f", BUILD_NUMBER);
	sprintf(strMisc, "Version: %s", BUILD_NUMBER);
	cairo_show_text(cr, strMisc);

	return FALSE;
}


gboolean on_drawWinch1_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	drawSpeedGauge (cr, 0, WINCH_TELEMETRY_SPEED_X, WINCH_TELEMETRY_SPEED_Y);
	drawLoadGauge (cr, 0, WINCH_TELEMETRY_LOAD_X, WINCH_TELEMETRY_LOAD_Y);
	drawPositionGauge (cr, 0, WINCH_TELEMETRY_POSITION_X, WINCH_TELEMETRY_POSITION_Y);
	drawTelemetryChart (cr, 0, WINCH_TELEMETRY_CHART_X, WINCH_TELEMETRY_CHART_Y);
	return FALSE;
}

gboolean on_drawWinch2_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	drawSpeedGauge (cr, 1, WINCH_TELEMETRY_SPEED_X, WINCH_TELEMETRY_SPEED_Y);
	drawLoadGauge (cr, 1, WINCH_TELEMETRY_LOAD_X, WINCH_TELEMETRY_LOAD_Y);
	drawPositionGauge (cr, 1, WINCH_TELEMETRY_POSITION_X, WINCH_TELEMETRY_POSITION_Y);
	drawTelemetryChart (cr, 1, WINCH_TELEMETRY_CHART_X, WINCH_TELEMETRY_CHART_Y);
	return FALSE;
}

gboolean on_drawWinch3_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	drawSpeedGauge (cr, 2, WINCH_TELEMETRY_SPEED_X, WINCH_TELEMETRY_SPEED_Y);
	drawLoadGauge (cr, 2, WINCH_TELEMETRY_LOAD_X, WINCH_TELEMETRY_LOAD_Y);
	drawPositionGauge (cr, 2, WINCH_TELEMETRY_POSITION_X, WINCH_TELEMETRY_POSITION_Y);
	drawTelemetryChart (cr, 2, WINCH_TELEMETRY_CHART_X, WINCH_TELEMETRY_CHART_Y);
	return FALSE;
}

gboolean on_drawWinch4_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	drawSpeedGauge (cr, 3, WINCH_TELEMETRY_SPEED_X, WINCH_TELEMETRY_SPEED_Y);
	drawLoadGauge (cr, 3, WINCH_TELEMETRY_LOAD_X, WINCH_TELEMETRY_LOAD_Y);
	drawPositionGauge (cr, 3, WINCH_TELEMETRY_POSITION_X, WINCH_TELEMETRY_POSITION_Y);
	drawTelemetryChart (cr, 3, WINCH_TELEMETRY_CHART_X, WINCH_TELEMETRY_CHART_Y);
	return FALSE;
}

//--- speedChart display
gboolean on_areaSpeedChart_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	int i,k;
	float SpeedVal, xSpeedVal, ySpeedVal, zSpeedVal, FrameDistVal;

	//paint background black
	cairo_set_line_width (cr, 1.0);
	cairo_set_source_rgba (cr, BLK);
	cairo_rectangle (cr, 1, 1, 543, 198);
	cairo_fill(cr);
	cairo_stroke(cr);

	cairo_translate(cr, 0, 100); //move "zero" point

	//for (i=0; i<motionDataFrames; i++)
	cairo_set_source_rgba (cr, RED);
	cairo_move_to (cr, 0, 0);
	k=0;
	for (i=0; i<(motionDataFrames-chartZoom); i+=chartZoom)
	{
		FrameDistVal = (motionData[i+1].xPosition - motionData[i].xPosition);
		SpeedVal = FrameDistVal * CPF_TO_MPS; //cm/frame to m/s
		cairo_line_to(cr, k, (SpeedVal * SPD_CHART_MULT));
		k++;
	}
	cairo_stroke(cr);

	cairo_set_source_rgba (cr, GRN);
	cairo_move_to (cr, 0, 0);
	k=0;
	for (i=0; i<(motionDataFrames-chartZoom); i+=chartZoom)
	{
		FrameDistVal = (motionData[i+1].yPosition - motionData[i].yPosition);
		SpeedVal = FrameDistVal * CPF_TO_MPS; //cm/frame to m/s
		cairo_line_to(cr, k, (SpeedVal * SPD_CHART_MULT));
		k++;
	}
	cairo_stroke(cr);

	cairo_set_source_rgba (cr, BLU);
	cairo_move_to (cr, 0, 0);
	k=0;
	for (i=0; i<(motionDataFrames-chartZoom); i+=chartZoom)
	{
		FrameDistVal = (motionData[i+1].zPosition - motionData[i].zPosition);
		SpeedVal = FrameDistVal * CPF_TO_MPS; //cm/frame to m/s
		cairo_line_to(cr, k, (SpeedVal * SPD_CHART_MULT));
		k++;
	}
	cairo_stroke(cr);

	//draw center, min, max lines
	cairo_set_line_width (cr, 0.5);
	cairo_set_source_rgba (cr, YEL01);	
	cairo_set_dash(cr, dash1, 2, 0);
	cairo_move_to (cr, 4, -50);
	cairo_line_to(cr, 541, -50);
	cairo_move_to (cr, 4, 0);
	cairo_line_to(cr, 541, 0);
	cairo_move_to (cr, 4, 50);
	cairo_line_to(cr, 541, 50);
	cairo_stroke(cr);
	
	cairo_set_source_rgba (cr, RED01);	
	cairo_move_to (cr, 4, -80);
	cairo_line_to(cr, 541, -80);
	cairo_move_to (cr, 4, 80);
	cairo_line_to(cr, 541, 80);
	cairo_stroke(cr);

	cairo_set_dash(cr, dash1, OFF, 0);
	
	//draw playback line
	k = mdcount/chartZoom;
	cairo_set_line_width (cr, 1.0);
	cairo_set_source_rgba (cr, BLU01);
	cairo_move_to (cr, k+1, -90);
	cairo_line_to(cr, k+1, 90);
	cairo_stroke(cr);
	
	//draw current axis speed values near playback line
	cairo_set_source_rgba (cr, RED);
	cairo_move_to(cr, k+10, -85);
	xSpeedVal = (motionData[mdcount+1].xPosition - motionData[mdcount].xPosition) * CPF_TO_MPS; 
	sprintf(strMisc, "X: %6.2f", xSpeedVal);
	cairo_show_text(cr, strMisc);
	cairo_set_source_rgba (cr, GRN);
	cairo_move_to(cr, k+10, -70);
	ySpeedVal = (motionData[mdcount+1].yPosition - motionData[mdcount].yPosition) * CPF_TO_MPS; 
	sprintf(strMisc, "Y: %6.2f", ySpeedVal);
	cairo_show_text(cr, strMisc);
	cairo_set_source_rgba (cr, BLU);
	cairo_move_to(cr, k+10, -55);
	zSpeedVal = (motionData[mdcount+1].zPosition - motionData[mdcount].zPosition) * CPF_TO_MPS; 
	sprintf(strMisc, "Z: %6.2f", zSpeedVal);
	cairo_show_text(cr, strMisc);
	

	//draw outline - do this last to cover up any other lines near border
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, BLU01);
	cairo_rectangle (cr, 1, -99, 543, 198);
	cairo_stroke(cr);
	
	cairo_set_source_rgba (cr, YEL01);	
	cairo_move_to(cr, 400, 90);
	sprintf(strMisc, "Pan Multiturn: %6.2f", motionData[mdcount].panMultiTurn);
	cairo_show_text(cr, strMisc);

	cairo_set_source_rgba (cr, YEL01);	
	cairo_move_to(cr, 10, 90);
	sprintf(strMisc, "Start: %s [%d]", motionData[startFrame-1].timecode,startFrame);
	cairo_show_text(cr, strMisc);

	cairo_set_source_rgba (cr, YEL01);	
	cairo_move_to(cr, 200, 90);
	sprintf(strMisc, "Current: %s [%d]", motionData[mdcount].timecode,mdcount+1);
	cairo_show_text(cr, strMisc);
	
	return FALSE;	
}

//--- draw XYZ display
gboolean on_areaXYZ_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	//main box
	cairo_set_source_rgba (cr, BLU01);
	cairo_set_line_width (cr, 1.5);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
	cairo_rectangle (cr, 1, 1, 598, 48);
	cairo_stroke(cr);


	cairo_set_font_size(cr, 25);
	cairo_move_to(cr, 10, 40);
	cairo_show_text(cr, "X:");
	cairo_move_to(cr, 210, 40);
	cairo_show_text(cr, "Y:");
	cairo_move_to(cr, 410, 40);
	cairo_show_text(cr, "Z:");

	cairo_set_font_size(cr, 30);
	cairo_set_source_rgba (cr, YEL01);
	cairo_move_to(cr, 50, 40);
	sprintf(strMisc, "%03.2f m", motionCommand.xPosition/100.0);
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, 250, 40);
	sprintf(strMisc, "%03.2f m", motionCommand.yPosition/100.0);
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, 450, 40);
	sprintf(strMisc, "%03.2f m", motionCommand.zPosition/100.0);
	cairo_show_text(cr, strMisc);

	return FALSE;
}

//--- draw elevation path
gboolean on_elevationDrawArea_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	int i;
	//main box
	cairo_set_source_rgba (cr, BLU01);
	cairo_set_line_width (cr, 1.5);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
	cairo_rectangle (cr, 1, 1, 598, 148);
	cairo_stroke(cr);

	cairo_translate(cr, 300, 150); //move "zero" point

	cairo_set_source_rgba (cr, YEL01);
	cairo_set_font_size(cr, 10);
	cairo_move_to(cr, -25, -128);
	sprintf(strMisc, "%3.2f cm", yPositionCurrent);
	cairo_show_text(cr, strMisc);

	
	//draw horizontal/vertical grid lines
	cairo_set_line_width (cr, 1.0);
	cairo_set_source_rgba (cr, BLU02);
	cairo_set_dash(cr, dash2, 2, 0);
	for (i = -250; i<=250; i+=50)
	{
		cairo_move_to (cr, i, -10);
		cairo_line_to (cr, i, -140);
	}
	for (i = -140; i<=-20; i+=20)
	{
		cairo_move_to (cr, -290, i);
		cairo_line_to (cr, 290, i);
	}
	cairo_stroke(cr);	
	cairo_set_dash(cr, dash2, OFF, 0);

	//vertical center line
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, BLU02);
	cairo_move_to (cr, 0, -5);
	cairo_line_to (cr, 0, -125);
	cairo_stroke(cr);

	cairo_set_source_rgba (cr, YEL02);
	cairo_set_font_size(cr, 8);
	cairo_move_to(cr, 275, -18);
	sprintf(strMisc, " 2 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -38);
	sprintf(strMisc, " 4 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -58);
	sprintf(strMisc, " 6 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -78);
	sprintf(strMisc, " 8 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -98);
	sprintf(strMisc, "10 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -118);
	sprintf(strMisc, "12 m");
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 275, -138);
	sprintf(strMisc, "14 m");
	cairo_show_text(cr, strMisc);

	//draw system boundary
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, RED01);
	cairo_set_dash(cr, dash1, 2, 0);
	cairo_rectangle (cr,
		systemBoundary.xmin/MP_SCALE, //corner x
		systemBoundary.ymax/-MP_SCALE, //corner y
		(systemBoundary.xmax - systemBoundary.xmin)/MP_SCALE,  //length x
		(systemBoundary.ymax - systemBoundary.ymin)/MP_SCALE); //length y
	cairo_stroke(cr);
	cairo_set_dash(cr, dash1, OFF, 0);

	//draw pulley points
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, RED01);
	for (int i = 0; i<4; i++)
	{
		cairo_arc(cr,pulleyLocation[i].xLocation/MP_SCALE,pulleyLocation[i].yLocation/-MP_SCALE,4,0,2*G_PI);
		cairo_stroke(cr);
	}


	//draw path
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, BLU01);
	if ((flashToggle == TRUE) && (pathWithinBoundary == FALSE))
	{
		cairo_set_source_rgba (cr, RED01);
	}

	cairo_move_to(cr, motionData[1].xPosition/MP_SCALE, motionData[1].yPosition/-MP_SCALE);
	for (int i=1; i<motionDataFrames; i++)
	{
		cairo_line_to(cr, motionData[i].xPosition/MP_SCALE, (motionData[i].yPosition - PTR_Y_OFFSET)/-MP_SCALE);
	}
	cairo_stroke(cr);

	//draww winch lines
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, YEL03);
	for (int i = 0; i<4; i++)
	{
		cairo_move_to (cr, pulleyLocation[i].xLocation/MP_SCALE, pulleyLocation[i].yLocation/-MP_SCALE);
		cairo_line_to (cr, motionCommand.xPosition/MP_SCALE,motionCommand.yPosition/-MP_SCALE);
	}
	cairo_stroke(cr);


	//draw character position
	cairo_set_line_width (cr, 1.5);
	cairo_set_source_rgba (cr, YEL01);
	cairo_rectangle (cr,
		(motionCommand.xPosition/MP_SCALE)-2, //corner x
		(motionCommand.yPosition/-MP_SCALE)+2, //corner y
		4, 6); //size
	cairo_move_to (cr, motionCommand.xPosition/MP_SCALE,(motionCommand.yPosition/-MP_SCALE)+2);
	cairo_line_to (cr, motionCommand.xPosition/MP_SCALE,(motionCommand.yPosition/-MP_SCALE)-10);
	cairo_rectangle (cr,
		(motionCommand.xPosition/MP_SCALE)-1.5, //corner x
		(motionCommand.yPosition/-MP_SCALE)-10, //corner y
		3, 3); //size


	//cairo_arc(cr,motionData[mdcount].xPosition/MP_SCALE, (motionData[mdcount].yPosition - PTR_Y_OFFSET)/-MP_SCALE,4,0,2*G_PI);
	cairo_stroke(cr);


	return FALSE;
}

//--- draw overhead view
gboolean on_floorDrawArea_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	int i;
	//main boxes
	cairo_set_source_rgba (cr, BLU01);
	cairo_set_line_width (cr, 1.5);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
	cairo_rectangle (cr, 1, 1, 598, 373);
	cairo_stroke(cr);

	cairo_translate(cr, 300, 187); //move "zero" point

	
	//draw horizontal/vertical grid lines
	
	cairo_set_line_width (cr, 0.75);
	cairo_set_source_rgba (cr, BLU02);
	cairo_set_dash(cr, dash2, 2, 0);
	for (i = -240; i<=240; i+=40)
	{
		cairo_move_to (cr, i, -177);
		cairo_line_to (cr, i, 177);
	}
	for (i = -140; i<=140; i+=40)
	{
		cairo_move_to (cr, -290, i);
		cairo_line_to (cr, 290, i);
	}
	cairo_stroke(cr);	
	cairo_set_dash(cr, dash2, OFF, 0);
	
	
	//draw floor orange grid lines
	cairo_set_line_width (cr, 0.35);
	cairo_set_source_rgba (cr, ORG01);
	for (i = -200; i<=200; i+=20)
	{
		cairo_move_to (cr, i, -90);
		cairo_line_to (cr, i, 90);
	}
	for (i = -90; i<=90; i+=20)
	{
		cairo_move_to (cr, -200, i);
		cairo_line_to (cr, 200, i);
	}
	cairo_stroke(cr);	

	cairo_set_line_width (cr, 1.0);
	cairo_set_source_rgba (cr, BLU02);


	//draw xz lines
	cairo_arc(cr,0,0,4,0,2*G_PI);
	cairo_set_line_width (cr, 2.0);
	cairo_move_to (cr, 0, -177);
	cairo_line_to (cr, 0, 170);
	cairo_move_to (cr, -290, 0);
	cairo_line_to (cr, 290, 0);
	cairo_stroke(cr);
	

	//draw system boundary
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, RED01);
	cairo_set_dash(cr, dash1, 2, 0);
	//cairo_rectangle (cr,
	//	systemBoundary.xmin/MP_SCALE, //corner x
	//	systemBoundary.zmin/MP_SCALE, //corner z
	//	(systemBoundary.xmax - systemBoundary.xmin)/MP_SCALE,  //length x
	//	(systemBoundary.zmax - systemBoundary.zmin)/MP_SCALE); //length z


	cairo_move_to (cr, systemBoundary.p1x/MP_SCALE, systemBoundary.p1z/MP_SCALE);
	cairo_line_to (cr, systemBoundary.p2x/MP_SCALE, systemBoundary.p2z/MP_SCALE);
	cairo_line_to (cr, systemBoundary.p3x/MP_SCALE, systemBoundary.p3z/MP_SCALE);
	cairo_line_to (cr, systemBoundary.p4x/MP_SCALE, systemBoundary.p4z/MP_SCALE);
	cairo_line_to (cr, systemBoundary.p1x/MP_SCALE, systemBoundary.p1z/MP_SCALE);

	
	cairo_stroke(cr);
	cairo_set_dash(cr, dash1, OFF, 0);

	//draw pulley points
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, RED01);
	for (i = 0; i<4; i++)
	{
		cairo_arc(cr,pulleyLocation[i].xLocation/MP_SCALE,pulleyLocation[i].zLocation/MP_SCALE,4,0,2*G_PI);
		cairo_stroke(cr);
	}


	//draw text
	cairo_set_source_rgba (cr, YEL01);
	cairo_set_font_size(cr, 10);
	cairo_move_to(cr, 235, -5);
	sprintf(strMisc, "%3.2f cm", xPositionCurrent);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, -25, 182);
	sprintf(strMisc, "%3.2f cm", zPositionCurrent);
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, -293, 180);
	sprintf(strMisc, "L0: %3.2f m", winchData[0].LineLength/100);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, -293, -170);
	sprintf(strMisc, "L1: %3.2f m", winchData[1].LineLength/100);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 235, -170);
	sprintf(strMisc, "L2: %3.2f m", winchData[2].LineLength/100);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 235, 180);
	sprintf(strMisc, "L3: %3.2f m", winchData[3].LineLength/100);
	cairo_show_text(cr, strMisc);

	//draw boundary value text
	cairo_set_source_rgba (cr, RED01);
	cairo_move_to(cr, systemBoundary.xmin/MP_SCALE-20, -100);
	sprintf(strMisc, "%3.1f m", systemBoundary.xmin/100);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, systemBoundary.xmax/MP_SCALE-20, -100);
	sprintf(strMisc, "%3.1f m", systemBoundary.xmax/100);
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, -250, systemBoundary.zmin/MP_SCALE);
	sprintf(strMisc, "%3.1f m", systemBoundary.zmin/100);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, -250, systemBoundary.zmax/MP_SCALE);
	sprintf(strMisc, "%3.1f m", systemBoundary.zmax/100);
	cairo_show_text(cr, strMisc);
	

	cairo_select_font_face(cr,"UbuntuMono",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, 25);
	if (systemStatus.Moving == TRUE)
	{
		cairo_set_source_rgba (cr, YEL01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, YEL02);
		}
		cairo_move_to(cr, -85, -160);
		sprintf(strMisc, "SYSTEM MOVING!");
		cairo_show_text(cr, strMisc);
	}
	else
	{
		cairo_set_source_rgba (cr, YEL02);
		cairo_move_to(cr, -85, -160);
		sprintf(strMisc, "SYSTEM STOPPED");
		cairo_show_text(cr, strMisc);
	}


	//draw path
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, BLU01);
	if ((flashToggle == TRUE) && (pathWithinBoundary == FALSE))
		{
			cairo_set_source_rgba (cr, RED01);
		}
	cairo_move_to(cr, motionData[0].xPosition/MP_SCALE, motionData[0].zPosition/MP_SCALE);

	for (i=0; i<motionDataFrames; i++)
	{
		cairo_line_to(cr, motionData[i].xPosition/MP_SCALE, motionData[i].zPosition/MP_SCALE);
	}
	cairo_stroke(cr);

	//draw winch lines
	cairo_set_line_width (cr, 2.0);
	cairo_set_source_rgba (cr, YEL03);
	for (i = 0; i<4; i++)
	{
		cairo_move_to (cr, pulleyLocation[i].xLocation/MP_SCALE, pulleyLocation[i].zLocation/MP_SCALE);
		cairo_line_to (cr, motionCommand.xPosition/MP_SCALE,motionCommand.zPosition/MP_SCALE);
	}
	cairo_stroke(cr);

	//draw PTR head command position
	cairo_set_line_width (cr, 1.5);
	cairo_set_source_rgba (cr, YEL01);
	cairo_rectangle (cr,
		(motionCommand.xPosition/MP_SCALE)-3, //corner x
		(motionCommand.zPosition/MP_SCALE)-3, //corner z
		6, 6); //size
	cairo_stroke(cr);

	//draw PTR head data position
	cairo_set_line_width (cr, 1.5);
	cairo_set_source_rgba (cr, YEL01);
	cairo_rectangle (cr,
		(motionData[mdcount].xPosition/MP_SCALE)-2, //corner x
		(motionData[mdcount].zPosition/MP_SCALE)-2, //corner z
		4, 4); //size
	cairo_stroke(cr);


	//cairo_arc(cr,motionData[mdcount].xPosition/MP_SCALE,motionData[mdcount].zPosition/MP_SCALE,4,0,2*G_PI);

	return FALSE;
}

//--- draw PTR telemetry
gboolean on_ptrDrawingArea_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	
	float iconRotation;
	//main boxes
	cairo_set_source_rgba (cr, BLU01);
	cairo_set_line_width (cr, 1.5);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
	cairo_rectangle (cr, 5, 15, 100, 100);
	cairo_rectangle (cr, 115, 15, 100, 100);
	cairo_rectangle (cr, 225, 15, 100, 100);
	cairo_stroke(cr);

	//timecode and data text display
	cairo_set_font_size(cr, 10);
	cairo_move_to(cr, 5, 10);
	sprintf(strMisc, "%s", motionData[mdcount].timecode);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 8, 25);
	sprintf(strMisc, "PAN: %3.2f", motionData[mdcount].panRotation);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 118, 25);
	sprintf(strMisc, "TILT: %3.2f", motionData[mdcount].tiltRotation);
	cairo_show_text(cr, strMisc);
	cairo_move_to(cr, 228, 25);
	sprintf(strMisc, "ROLL: %3.2f", motionData[mdcount].rollRotation);
	cairo_show_text(cr, strMisc);
	//cairo_stroke(cr);


	//progress line display
	cairo_rectangle (cr, 80, 1, 100, 8);
	cairo_stroke(cr);
	int x = (int)((float)mdcount * (100.0/motionDataFrames));
	cairo_rectangle (cr, 80, 2, x, 6);
	cairo_fill(cr);

	//draw rotated ipad icon based on motion data - pan axis
	cairo_translate(cr, 55, 65); //move "zero" point
	cairo_set_source_rgba (cr, YEL01);
	iconRotation = (90 + motionData[mdcount].panRotation) * (G_PI/180.0) * -1;
	cairo_rotate (cr, iconRotation);
	cairo_move_to (cr, 0, 0);
	cairo_line_to (cr, -10, -10);
	cairo_move_to (cr, 0, 0);
	cairo_line_to (cr, -10, 10);
	cairo_move_to (cr, 4, 0);
	cairo_arc(cr,0,0,4,0,2*G_PI);
	cairo_stroke (cr);
	cairo_rectangle (cr,-16,-20, 8, 40);
	cairo_fill (cr);
	cairo_set_source_rgba (cr, BLU02);
	cairo_rectangle (cr,-16,-16, 4, 32);
	cairo_fill (cr);

	//draw rotated ipad icon based on motion data - tilt axis
	cairo_rotate (cr, -1*iconRotation); //return rotation back to zero
	cairo_set_source_rgba (cr, YEL01);
	cairo_translate(cr, 110, 0); //move "zero" point
	iconRotation = motionData[mdcount].tiltRotation * (G_PI/180.0) * -1;
	cairo_rotate (cr, iconRotation);
	cairo_move_to (cr, 0, 0);
	cairo_line_to (cr, -10, -10);
	cairo_move_to (cr, 0, 0);
	cairo_line_to (cr, -10, 10);
	cairo_move_to (cr, 4, 0);
	cairo_arc(cr,0,0,4,0,2*G_PI);
	cairo_stroke (cr);
	cairo_rectangle (cr,-16,-30, 8, 60);
	cairo_fill (cr);
	cairo_set_source_rgba (cr, BLU02);
	cairo_rectangle (cr,-16,-26, 4, 52);
	cairo_fill (cr);

	//draw rotated ipad icon based on motion data - roll axis
	cairo_rotate (cr, -1*iconRotation); //return rotation back to zero
	cairo_set_source_rgba (cr, YEL01);
	cairo_set_line_width (cr, 2);
	cairo_translate(cr, 110, 0); //move "zero" point
	iconRotation = motionData[mdcount].rollRotation * (G_PI/180.0) * -1;
	cairo_rotate (cr, iconRotation);
	cairo_move_to (cr, -20, -30);
	cairo_line_to (cr, 20, 30);
	cairo_move_to (cr, 20, -30);
	cairo_line_to (cr, -20, 30);
	cairo_move_to (cr, 4, 0);
	cairo_arc(cr,0,0,4,0,2*G_PI);
	cairo_stroke (cr);
	cairo_rectangle (cr,-20,-30, 40, 60);
	cairo_stroke (cr);
	cairo_set_source_rgba (cr, 0.0,0.3,0.5,0.3);
	cairo_rectangle (cr,-18,-28, 36, 56);
	cairo_fill (cr);




	return FALSE;
}

//draw winch telemetry chart and winch status messages
void drawTelemetryChart (cairo_t *cr, int wnum, int xpos, int ypos)
{
	int i=0, j=0;

	cairo_set_line_width (cr, 1.5);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);

	cairo_set_font_size(cr, 40);
	cairo_set_source_rgba (cr, YEL02);
	cairo_move_to(cr, 15, 65);
	sprintf(strMisc, "%02d", wnum+1);
	cairo_show_text(cr, strMisc);

	cairo_set_font_size(cr, 15);
	cairo_set_source_rgba (cr, RED01);
	cairo_move_to(cr, 555, 40);
	sprintf(strMisc, "%05d", commErrorCountTotal[wnum]);
	cairo_show_text(cr, strMisc);



	cairo_select_font_face(cr,"UbuntuMono",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, 16);
	cairo_move_to(cr, 20, 85);
	if (winchStatus[wnum].Comms == ERROR)
	{
		cairo_set_source_rgba (cr, RED02);
		sprintf(strMisc, "SERVO ??");		
	}
	else if (winchStatus[wnum].Servo == ON)
	{
		cairo_set_source_rgba (cr, YEL01);
		sprintf(strMisc, "SERVO ON");
	}
	else
	{
		cairo_set_source_rgba (cr, YEL02);
		sprintf(strMisc, "SERVO OFF");
	}
	cairo_show_text(cr, strMisc);
	
	cairo_move_to(cr, 20, 105);
	if (winchStatus[wnum].Comms == ERROR)
	{
		cairo_set_source_rgba (cr, RED02);
		sprintf(strMisc, "BRAKE ??");		
	}
	else if (winchStatus[wnum].Brake == ON)
	{
		cairo_set_source_rgba (cr, YEL02);
		sprintf(strMisc, "BRAKE ON");
	}
	else
	{
		cairo_set_source_rgba (cr, YEL01);
		sprintf(strMisc, "BRAKE OFF");
	}
	cairo_show_text(cr, strMisc);

	cairo_move_to(cr, 20, 125);
	if (winchStatus[wnum].Comms == ERROR)
	{
		cairo_set_source_rgba (cr, RED02);
		sprintf(strMisc, "ALARM ??");		
	}
	else if (winchStatus[wnum].Alarm == ON)
	{
		cairo_set_source_rgba (cr, RED01);
		sprintf(strMisc, "ALARM ON");
	}
	else
	{
		cairo_set_source_rgba (cr, YEL02);
		sprintf(strMisc, "ALARM OFF");
	}
	cairo_show_text(cr, strMisc);


	cairo_set_font_size(cr, 25);
	//display COMM status
	if (winchStatus[wnum].Comms == OK)
	{
		cairo_set_source_rgba (cr, YEL01);
		//if (flashToggle == TRUE)
		//{
		//	cairo_set_source_rgba (cr, YEL02);
		//}
		cairo_move_to(cr, 20, 155);
		sprintf(strMisc, "COMM");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 20, 180);
		sprintf(strMisc, "OK");
		cairo_show_text(cr, strMisc);
	}
	else
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		cairo_move_to(cr, 20, 155);
		sprintf(strMisc, "COMMS");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 20, 180);
		sprintf(strMisc, "ALARM");
		cairo_show_text(cr, strMisc);
	}
	
	cairo_move_to(cr, 0, 0);

	//draw chart timelines
	cairo_set_font_size(cr, 8);
	cairo_set_source_rgba (cr, BLU02);
	for (i = 0; i < 5; i++)
	{
		if ((i > 0) || (chartTimeCounter < 85)) //sets appearance at end of chart
		{
			cairo_move_to (cr, chartTimeCounter+400+xpos-(100*i), ypos-25);
			cairo_line_to (cr, chartTimeCounter+400+xpos-(100*i), ypos+20);
			cairo_move_to(cr, chartTimeCounter+390+xpos-(100*i), ypos+28);
		}
		if ((i < 4) || (chartTimeCounter > 10)) //keeps left end time from running past chart area
		{
			int tempTime = chartTimeSeconds - (5*i);
			sprintf(strMisc, "%02d:%02d", tempTime/60,tempTime%60);
			cairo_show_text(cr, strMisc);
		}
	}
	cairo_stroke (cr);

	//draw chart data
	j=0;
	cairo_move_to (cr, j+xpos, (winchChartLoadData[wnum][0]/6)+ypos);
	for (i=0;i<=485;i++)
	{
		cairo_set_source_rgba (cr, BLU01);
		cairo_line_to (cr, j+xpos, (winchChartLoadData[wnum][i]/6)+ypos);
		j++;
	}
	cairo_stroke (cr);
	
	j=0;
	cairo_move_to (cr, j+xpos, (winchChartSpeedData[wnum][0]/600)+ypos);
	for (i=0;i<=485;i++)
	{
		cairo_set_source_rgba (cr, YEL01);
		cairo_line_to (cr, j+xpos, (winchChartSpeedData[wnum][i]/600)+ypos);
		j++;
	}
	cairo_stroke (cr);

	j=0;
	cairo_move_to (cr, j+xpos, (winchChartSpeedCommandData[wnum][0]/600)+ypos); //600 is normal value
	for (i=0;i<=485;i++)
	{
		cairo_set_source_rgba (cr, YEL02);
		cairo_line_to (cr, j+xpos, (winchChartSpeedCommandData[wnum][i]/600)+ypos);
		j++;
	}
	cairo_stroke (cr);
}


void drawSpeedGauge (cairo_t *cr, int wnum, int xpos, int ypos)
{
	float rotpos;
	float gaugePosition = abs(winchData[wnum].speedCommand) / 22.22; //3000 rpm to 135 degrees
	float rpos_command = gaugePosition * (G_PI/180.0);
	gaugePosition = abs(winchData[wnum].speedActual) / 22.22;
	float rpos_actual = gaugePosition * (G_PI/180.0);

	//text data
	cairo_set_font_size(cr, 18);
	cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	cairo_move_to(cr, xpos-28, ypos+3);
	sprintf(strMisc, "%04d rpm", winchData[wnum].speedActual);
	cairo_show_text(cr, strMisc);

	//command value
	cairo_set_font_size(cr, 12);
	cairo_move_to(cr, xpos-18, ypos-15);
	sprintf(strMisc, "%04d rpm", winchData[wnum].speedCommand);
	//sprintf(strMisc, "%06.3f mS", avgTime2);
	cairo_show_text(cr, strMisc);
	//cairo_stroke(cr);

	//draw small "command" line
	cairo_translate(cr, xpos, ypos); //move "zero" to gauge needle center
	cairo_set_line_width (cr, 2);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
  	cairo_rotate (cr, rpos_command);  //rotate to reading position
	cairo_move_to (cr, -35, 0);
	cairo_line_to (cr, -43, 0);
	cairo_stroke (cr);

	cairo_rotate (cr, -rpos_command); //reset rotation
  	rpos_actual = (int)(rpos_actual*5); //create granularity for boxes display
	rpos_actual = rpos_actual/5.0;

	//draw an initial rectangle (for zero)
	cairo_rectangle (cr, -62, -6, 12, 6);
	rotpos = 0.2;
	cairo_rotate (cr, rotpos);
	cairo_fill(cr);

	//draw actual value rectangles
	//change color to red if gauge is past 90 degrees
	for (int i = 0; i < rpos_actual * 5; i++) {
	if (i < 7)
	{
		cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	}
	else
	{
		cairo_set_source_rgba (cr, 255/255.0, 0/255.0, 0/255.0, 1.0);
	}
		cairo_rectangle (cr, -62, -6, 12, 6);
		rotpos += 0.2;
		cairo_rotate (cr, 0.2);
		cairo_fill(cr);
	}
	//cairo_close_path(cr);
	cairo_rotate (cr, -rotpos);
	cairo_translate(cr, -xpos, -ypos); //move "zero" to gauge needle center


}

void drawLoadGauge (cairo_t *cr, int wnum, int xpos, int ypos)
{
	float rotpos;
	float gaugePosition = fabs(winchData[wnum].motorLoad) / 1.11; //150 percent converted to 135 degrees
	float rpos_load = gaugePosition * (G_PI/180.0);
	gaugePosition = fabs(winchData[wnum].motorLoadMax) / 1.11; //convert to 135 degrees
	float rpos_max = gaugePosition * (G_PI/180.0);
	
	if (rpos_load > 2.356) //135 degrees in radians
		rpos_load = 2.356;
	if (rpos_max > 2.356)
		rpos_max = 2.356;

	//text data
	cairo_set_font_size(cr, 18);
	cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	cairo_move_to(cr, xpos-20, ypos+3);
	sprintf(strMisc, "%02.0f%%", winchData[wnum].motorLoad);
	cairo_show_text(cr, strMisc);

	//max load value
	cairo_set_font_size(cr, 12);
	cairo_move_to(cr, xpos-16, ypos-15);
	sprintf(strMisc, "%02.0f%%", winchData[wnum].motorLoadMax);
	cairo_show_text(cr, strMisc);

	//change color to red if gauge is past 90 degrees
	//draw small "command" line
	cairo_translate(cr, xpos, ypos); //move "zero" to gauge needle center
	cairo_set_line_width (cr, 2);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
  	cairo_rotate (cr, rpos_max);  //rotate to reading position
	cairo_move_to (cr, -35, 0);
	cairo_line_to (cr, -43, 0);
	cairo_stroke (cr);

	cairo_rotate (cr, -1*(rpos_max)); //reset rotation
  	rpos_load = (int)(rpos_load*5); //create granularity for boxes display
	rpos_load = rpos_load/5.0;

	//draw an initial rectangle (for zero)
	cairo_rectangle (cr, -62, -6, 12, 6);
	rotpos = 0.2;
	cairo_rotate (cr, rotpos);
	cairo_fill(cr);

	for (int i = 0; i < rpos_load * 5; i++) {
	if (i < 7)
	{
		cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	}
	else
	{
		cairo_set_source_rgba (cr, 255/255.0, 0/255.0, 0/255.0, 1.0);
	}
		cairo_rectangle (cr, -62, -6, 12, 6);
		rotpos += 0.2;
		cairo_rotate (cr, 0.2);
		cairo_fill(cr);
	}
	//cairo_close_path(cr);
	cairo_rotate (cr, -rotpos);
	cairo_translate(cr, -xpos, -ypos); //move "zero" to gauge needle center
}


void drawPositionGauge (cairo_t *cr, int wnum, int xpos, int ypos)
{
	//float rotpos;
	float gaugePosition;
	float rpos;

	gaugePosition = winchData[wnum].PositionError * 4.44; //+/-10 converted to +/- 45 degrees
	rpos = gaugePosition * (G_PI/180.0);
	rpos = rpos + (G_PI/2); //rotate 90 degrees CW to start position

	if (rpos > 2.356) //135 degrees in radians
		rpos = 2.356;
	if (rpos < 0.785) //45 degrees in radians (from horz)
		rpos = 0.785;

	//text data
	cairo_set_font_size(cr, 16);
	cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	cairo_move_to(cr, xpos-32, ypos-9);
	sprintf(strMisc, "%06.3f", winchData[wnum].PositionError);
	//sprintf(strMisc, "%0d error", commErrorCount);
	cairo_show_text(cr, strMisc);

	//Position Error
	cairo_set_font_size(cr, 10);
	cairo_move_to(cr, xpos-18, ypos-26);
	sprintf(strMisc, "%06.3f", winchData[wnum].PositionErrorMax);
	cairo_show_text(cr, strMisc);

	//change color to red if gauge is past 90 degrees
	//draw small "command" line
	
	cairo_translate(cr, xpos, ypos); //move "zero" to gauge needle center
	cairo_set_line_width (cr, 4);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
  	cairo_rotate (cr, rpos);  //rotate to reading position
	cairo_move_to (cr, -51, 0);
	cairo_line_to (cr, -60, 0);
	cairo_stroke (cr);

	cairo_rotate (cr, -1*(rpos)); //reset rotation
  	rpos = (int)(rpos*5); //create granularity for boxes display
	rpos = rpos/5.0;

/*	//draw an initial rectangle (for zero)
	cairo_rectangle (cr, -62, -6, 12, 6);
	rotpos = 0.2;
	cairo_rotate (cr, rotpos);
	cairo_fill(cr);

	for (int i = 0; i < rpos * 5; i++) {
	if (i < 7)
	{
		cairo_set_source_rgba (cr, 255/255.0, 174/255.0, 0/255.0, 1.0);
	}
	else 
	{
		cairo_set_source_rgba (cr, 255/255.0, 0/255.0, 0/255.0, 1.0);
	}
		cairo_rectangle (cr, -62, -6, 12, 6);
		rotpos += 0.2;
		cairo_rotate (cr, 0.2);
		cairo_fill(cr);
	}
	//cairo_close_path(cr);
*/	
	//cairo_rotate (cr, -rotpos);
	cairo_translate(cr, -xpos, -ypos); //move "zero" to gauge needle center

}


gboolean on_drawingarea1_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	int i, j;
	
	//draw chart data
	cairo_set_source_rgba (cr, YEL01);
	cairo_translate(cr, 0, 25); 

	j=0;
	cairo_move_to (cr, j, threadChartData[0]);
	for (i=0;i<600;i++)
	{
		cairo_line_to (cr, j, threadChartData[i]);
		j++;
	}
	cairo_stroke (cr);
 
 
	return FALSE;
}

gboolean on_areaStatus_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	cairo_select_font_face(cr,"UbuntuMono",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, 20);
	
	
	if (systemStatus.Comms == ERROR)
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM ERROR");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "COMM ALARM");
		cairo_show_text(cr, strMisc);
	}
	else if (systemStatus.PositionChange == ERROR)
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM ERROR");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "EXCESSIVE POSITION CHANGE");
		cairo_show_text(cr, strMisc);
	}
	else if (systemStatus.Alarm == ERROR)
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM ERROR");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "SERVO ALARM");
		cairo_show_text(cr, strMisc);
	}
	else if (systemStatus.Servo == ERROR)
	{
		cairo_set_source_rgba (cr, YEL01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, YEL02);
		}

		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM WARNING");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "ONE OR MORE SERVOS ARE OFF");
		cairo_show_text(cr, strMisc);
	}
	else if (systemBoundary.floor == OFF)
	{
		cairo_set_source_rgba (cr, YEL01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, YEL02);
		}

		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM WARNING");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "FLOOR BOUNDARY OFF");
		cairo_show_text(cr, strMisc);
	}
	else if (simulationTest == TRUE)
	{
		cairo_set_source_rgba (cr, BLU01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, BLU02);
		}

		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM OK");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "RUNNING SIMULATION TEST");
		cairo_show_text(cr, strMisc);
	}
	else
	{
		cairo_set_source_rgba (cr, BLU01);
		cairo_move_to(cr, 10, 23);
		sprintf(strMisc, "SYSTEM OK");
		cairo_show_text(cr, strMisc);

		cairo_move_to(cr, 10, 43);
		sprintf(strMisc, "READY TO OPERATE");
		cairo_show_text(cr, strMisc);
	}

	//draw box same color as text
	cairo_set_line_width (cr, 5.0);
	cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
	cairo_rectangle (cr, 1, 1, 298, 48);
	cairo_stroke(cr);
	
	
 
	return FALSE;
}

gboolean on_areaServoStatus_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	cairo_select_font_face(cr,"UbuntuMono",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);



	cairo_set_font_size(cr, 30);
	cairo_move_to(cr, 40, 35);
	if (setServoOnOff == ON)
	{
		cairo_set_source_rgba (cr, YEL01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, YEL02);
		}
		sprintf(strMisc, "SERVOS ON");	
	}
	else
	{
		cairo_set_source_rgba (cr, YEL02);
		sprintf(strMisc, "SERVOS OFF");			
	}
	cairo_show_text(cr, strMisc);

	return FALSE;
}

gboolean on_areaAlarmStatus_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	cairo_select_font_face(cr,"UbuntuMono",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, 30);
	cairo_move_to(cr, 20, 35);
	if (systemStatus.Alarm == ON)
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		sprintf(strMisc, "WINCH ALARM");	
	}
	else if (systemStatus.Comms == ERROR)
	{
		cairo_set_source_rgba (cr, RED01);
		if (flashToggle == TRUE)
		{
			cairo_set_source_rgba (cr, RED02);
		}
		sprintf(strMisc, "COMMS ALARM");	
	}
	else
	{
		cairo_set_source_rgba (cr, YEL02);
		sprintf(strMisc, " NO ALARMS");			
	}
	cairo_show_text(cr, strMisc);
	return FALSE;
}

int16_t getMotorDataValue (int startByte)
{
	strMisc[0] = readBuffer[startByte];
	strMisc[1] = readBuffer[startByte+1];
	strMisc[2] = readBuffer[startByte+2];
	strMisc[3] = readBuffer[startByte+3];
	strMisc[4] = 0; //string terminator

	int16_t dataValue = (int16_t)strtol(strMisc, NULL, 16); //use base 16 (Hex) for string conversion
	return dataValue;
}

void getWinchStatus (int wnum)
{
	strMisc[0] = readBuffer[29];
	strMisc[1] = readBuffer[30];
	strMisc[2] = 0; //string terminator
	int dataValue = (int16_t)strtol(strMisc, NULL, 16); //use base 16 (Hex) for string conversion
	
	if ((dataValue & 0x01) == 0x01)
	{
		winchStatus[wnum].Brake = OFF; //backwards for this
	}
	else
	{
		winchStatus[wnum].Brake = ON;
	}
	
	if ((dataValue & 0x02) == 0x02)
	{
		winchStatus[wnum].Servo = ON;
	}
	else
	{
		winchStatus[wnum].Servo = OFF;
	}
	
	if ((dataValue & 0x04) == 0x04)
	{
		winchStatus[wnum].AlarmCounter++;
	}
	
	if (winchStatus[wnum].AlarmCounter > 5)
	{
		winchStatus[wnum].Alarm = ON;
		winchStatus[wnum].AlarmCounter = 100;
	}
	
	if ((dataValue & 0x04) == 0x00)
	{
		winchStatus[wnum].Alarm = OFF;
		winchStatus[wnum].AlarmCounter = 0;		
	}
	
	if ((dataValue & 0x08) == 0x08)
	{
		winchStatus[wnum].Overload = ON;
	}
	else
	{
		winchStatus[wnum].Overload = OFF;
	}
	
	if ((dataValue & 0x10) == 0x10)
	{
		winchStatus[wnum].VelocityReached = ON;
	}
	else
	{
		winchStatus[wnum].VelocityReached = OFF;
	}
	
}

void on_btnFloorOnOff_clicked ()
{
	if (systemBoundary.floor == ON)
	{
		systemBoundary.floor = OFF;
	}
	else
	{
		systemBoundary.floor = ON;
	}
}

//-----------------------------------------------------------------------------
//-----------------------  PLAYBACK MOTION ------------------------------------
//-----------------------------------------------------------------------------

void on_btnPlayBackPlay_clicked()
{
	if ((systemMode == MODE_AUTO) && (systemStatus.Master == OK))
	{
		if ((playbackMode == PLAYBACK_MODE_STOP) && 
			(systemAtPlayPosition == TRUE) && 
			(systemAtPlayBackEnd != TRUE))
		{
			playbackMode = PLAYBACK_MODE_PLAY;
		}
	}
}

void on_btnPlayBackStop_clicked()
{
	if (playbackMode == PLAYBACK_MODE_PLAY)
	{
		playbackMode = PLAYBACK_MODE_DECEL;
		playbackDecelValue = 1;
		playbackDecelCount = 0;
	}
}

void on_btnGoToStart_clicked ()
{
	//if start is within boundary
	//if position is at start
	mdcount = startFrame;
	
	if ((systemMode == MODE_AUTO) && (systemStatus.Master == OK))
	{
		playbackMode = PLAYBACK_MODE_GOTOSTART;
		gotoStartMode = GOTOSTART_MOVING;
		gotoStartAtX = FALSE;
		gotoStartAtY = FALSE;
		gotoStartAtZ = FALSE;
		gotoStartXSpeed = 0;
		gotoStartYSpeed = 0;
		gotoStartZSpeed = 0;
	}
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


void on_btnPTRreset_clicked()
{
	mdcount = startFrame;
	togglePTRplay = OFF;
	toggleMoveToZero = ON;
}


void on_btnResetPositionError_clicked ()
{
	systemStatus.PositionChange = OK;
	togglePTRplay = OFF;
	int i;
	for (i=0; i<4; i++)
	{
		winchData[i].LLoffset = winchData[i].LineLength;
		winchData[i].PAoffset = winchData[i].PositionActual;
	}
}


void on_btnLock_clicked ()
{
	if (systemStatus.Moving == FALSE)
	{
		systemMode = MODE_LOCKED;
		playbackMode = PLAYBACK_MODE_STOP;
		
		//setServoOnOff = OFF;
		//sendServoOnOff = TRUE;	
		//sendServoOnOffCounter = 0;
	}
}

void on_btnAuto_clicked ()
{
	if (systemStatus.Moving == FALSE)
	{
		systemMode = MODE_AUTO;
		playbackMode = PLAYBACK_MODE_STOP;
	}
}

void on_btnManual_clicked ()
{
	if (systemStatus.Moving == FALSE)
	{
		systemMode = MODE_MANUAL;
		playbackMode = PLAYBACK_MODE_STOP;
	}
}

void on_btnAlarmReset_clicked ()
{	
	sendAlarmReset = TRUE;	
	sendAlarmResetCounter = 0;
}

void on_btnServoOn_clicked ()
{
	if ((systemStatus.Comms == OK) && (systemStatus.Alarm == OFF) && (systemMode != MODE_LOCKED))
	{
		setServoOnOff = ON;
		sendServoOnOff = TRUE;
		sendServoOnOffCounter = 0;
	}
}

void on_btnServoOff_clicked ()
{
	setServoOnOff = OFF;
	sendServoOnOff = TRUE;	
	sendServoOnOffCounter = 0;
}

void on_btnResetLoad1_clicked ()
{
	winchData[0].motorLoadMax = 0;
	winchData[0].PositionErrorMax = 0;
}
void on_btnResetLoad2_clicked ()
{
	winchData[1].motorLoadMax = 0;
	winchData[1].PositionErrorMax = 0;
}
void on_btnResetLoad3_clicked ()
{
	winchData[2].motorLoadMax = 0;
	winchData[2].PositionErrorMax = 0;
}

void on_btnResetLoad4_clicked ()
{
	winchData[3].motorLoadMax = 0;
	winchData[3].PositionErrorMax = 0;
}

void on_cbPlaybackSpeed_changed() 
{
	switch (gtk_combo_box_get_active(g_cbPlaybackSpeed))
	{
		case 0:
			playbackSpeedPercentage = 100;
			break;
		case 1:
			playbackSpeedPercentage = 50;
			break;
		case 2:
			playbackSpeedPercentage = 25;
			break;
		case 3:
			playbackSpeedPercentage = 10;
			break;
		case 4:
			playbackSpeedPercentage = 5;
			break;
	}
}

void on_btnZoomIn_clicked ()
{
	if (chartZoom > 1)
	{
		chartZoom--;
	}
}

void on_btnZoomOut_clicked ()
{
	if (chartZoom < 10)
	{
		chartZoom++;
	}
}



//=============================================================================
//                              DIALOG BOXES
//=============================================================================

void on_btnBoundarySet_clicked ()
{
	//const gchar *setCurrentPositionX;
	//const gchar *setCurrentPositionY;
	//const gchar *setCurrentPositionZ;

	const gchar *strEntryValue;
	
	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.xmin/100.0);
	gtk_entry_set_text(g_setBoundaryXmin,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.xmax/100.0);
	gtk_entry_set_text(g_setBoundaryXmax,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.ymin/100.0);
	gtk_entry_set_text(g_setBoundaryYmin,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.ymax/100.0);
	gtk_entry_set_text(g_setBoundaryYmax,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.zmin/100.0);
	gtk_entry_set_text(g_setBoundaryZmin,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", systemBoundary.zmax/100.0);
	gtk_entry_set_text(g_setBoundaryZmax,strEntryValue);

	gtk_dialog_run(GTK_DIALOG(g_dlgBoundary));
	gtk_widget_hide(g_dlgBoundary);
}
void on_dlgBoundary_response (GtkDialog *dialog, gint response_id, gpointer user_data)
{
	const gchar *setXmin;
	const gchar *setXmax;
	const gchar *setYmin;
	const gchar *setYmax;
	const gchar *setZmin;
	const gchar *setZmax;
	
	if (response_id == 2) //cancel
	{
		//printf ("cancel position enter/n");
	}
	
	if (response_id == 1) //enter
	{
		setXmin = gtk_entry_get_text(g_setBoundaryXmin);
		setXmax = gtk_entry_get_text(g_setBoundaryXmax);
		setYmin = gtk_entry_get_text(g_setBoundaryYmin);
		setYmax = gtk_entry_get_text(g_setBoundaryYmax);
		setZmin = gtk_entry_get_text(g_setBoundaryZmin);
		setZmax = gtk_entry_get_text(g_setBoundaryZmax);
		
		char *ptr;
		systemBoundary.xmin = strtof(setXmin,&ptr)*100.0;
		systemBoundary.xmax = strtof(setXmax,&ptr)*100.0;
		systemBoundary.ymin = strtof(setYmin,&ptr)*100.0;
		systemBoundary.ymax = strtof(setYmax,&ptr)*100.0;
		systemBoundary.zmin = strtof(setZmin,&ptr)*100.0;
		systemBoundary.zmax = strtof(setZmax,&ptr)*100.0;

		systemBoundary.yminSaved = systemBoundary.ymin;
		
		g_key_file_set_double (gkfd,"boundary","boundaryXmin",systemBoundary.xmin);
		g_key_file_set_double (gkfd,"boundary","boundaryXmax",systemBoundary.xmax);
		g_key_file_set_double (gkfd,"boundary","boundaryYmin",systemBoundary.ymin);
		g_key_file_set_double (gkfd,"boundary","boundaryYmax",systemBoundary.ymax);
		g_key_file_set_double (gkfd,"boundary","boundaryZmin",systemBoundary.zmin);
		g_key_file_set_double (gkfd,"boundary","boundaryZmax",systemBoundary.zmax);
		
		g_key_file_save_to_file(gkfd,CONFIG_FILE_NAME,NULL);

	}	
}


void on_btnCurrentPosition_clicked ()
{
	const gchar *strEntryValue;
	
	strEntryValue = g_strdup_printf("%3.2f", xPositionCurrent/100.0);
	gtk_entry_set_text(g_setCurrentPositionX,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", yPositionCurrent/100.0);
	gtk_entry_set_text(g_setCurrentPositionY,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", zPositionCurrent/100.0);
	gtk_entry_set_text(g_setCurrentPositionZ,strEntryValue);
	
	gtk_dialog_run(GTK_DIALOG(g_dlgCurrentPosition));
	gtk_widget_hide(g_dlgCurrentPosition);
}
void on_dlgCurrentPosition_response (GtkDialog *dialog, gint response_id, gpointer user_data)
{
	const gchar *setCurrentPositionX;
	const gchar *setCurrentPositionY;
	const gchar *setCurrentPositionZ;
	
	if (response_id == 2) //cancel
	{
		//printf ("cancel position enter/n");
	}
	
	if (response_id == 1) //enter
	{
		setCurrentPositionX = gtk_entry_get_text(g_setCurrentPositionX);
		setCurrentPositionY = gtk_entry_get_text(g_setCurrentPositionY);
		setCurrentPositionZ = gtk_entry_get_text(g_setCurrentPositionZ);
		
		char *ptr;
		xPositionStartUp = strtof(setCurrentPositionX,&ptr)*100.0;
		yPositionStartUp = strtof(setCurrentPositionY,&ptr)*100.0;
		zPositionStartUp = strtof(setCurrentPositionZ,&ptr)*100.0;

		xPositionCurrent = xPositionStartUp;
		yPositionCurrent = yPositionStartUp;
		zPositionCurrent = zPositionStartUp;
		
		setCurrentPositionLockout = TRUE;
	}	
}

void on_btnSurvey_clicked ()
{
	const gchar *strEntryValue;

	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[0].xLocation/100.0);
	gtk_entry_set_text(g_setSurveyP1X,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[0].yLocation/100.0);
	gtk_entry_set_text(g_setSurveyP1Y,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[0].zLocation/100.0);
	gtk_entry_set_text(g_setSurveyP1Z,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[1].xLocation/100.0);
	gtk_entry_set_text(g_setSurveyP2X,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[1].yLocation/100.0);
	gtk_entry_set_text(g_setSurveyP2Y,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[1].zLocation/100.0);
	gtk_entry_set_text(g_setSurveyP2Z,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[2].xLocation/100.0);
	gtk_entry_set_text(g_setSurveyP3X,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[2].yLocation/100.0);
	gtk_entry_set_text(g_setSurveyP3Y,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[2].zLocation/100.0);
	gtk_entry_set_text(g_setSurveyP3Z,strEntryValue);

	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[3].xLocation/100.0);
	gtk_entry_set_text(g_setSurveyP4X,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[3].yLocation/100.0);
	gtk_entry_set_text(g_setSurveyP4Y,strEntryValue);
	strEntryValue = g_strdup_printf("%3.2f", pulleyLocation[3].zLocation/100.0);
	gtk_entry_set_text(g_setSurveyP4Z,strEntryValue);

	gtk_dialog_run(GTK_DIALOG(g_dlgSurvey));
	gtk_widget_hide(g_dlgSurvey);
}

void on_dlgSurvey_response (GtkDialog *dialog, gint response_id, gpointer user_data)
{
	const gchar *surveyValueP1X;
	const gchar *surveyValueP1Y;
	const gchar *surveyValueP1Z;
	const gchar *surveyValueP2X;
	const gchar *surveyValueP2Y;
	const gchar *surveyValueP2Z;
	const gchar *surveyValueP3X;
	const gchar *surveyValueP3Y;
	const gchar *surveyValueP3Z;
	const gchar *surveyValueP4X;
	const gchar *surveyValueP4Y;
	const gchar *surveyValueP4Z;

	if (response_id == 2) //cancel
	{
		//printf ("cancel survey enter/n");
	}
	
	if (response_id == 1) //enter
	{
		surveyValueP1X = gtk_entry_get_text(g_setSurveyP1X);
		surveyValueP1Y = gtk_entry_get_text(g_setSurveyP1Y);
		surveyValueP1Z = gtk_entry_get_text(g_setSurveyP1Z);
		surveyValueP2X = gtk_entry_get_text(g_setSurveyP2X);
		surveyValueP2Y = gtk_entry_get_text(g_setSurveyP2Y);
		surveyValueP2Z = gtk_entry_get_text(g_setSurveyP2Z);
		surveyValueP3X = gtk_entry_get_text(g_setSurveyP3X);
		surveyValueP3Y = gtk_entry_get_text(g_setSurveyP3Y);
		surveyValueP3Z = gtk_entry_get_text(g_setSurveyP3Z);
		surveyValueP4X = gtk_entry_get_text(g_setSurveyP4X);
		surveyValueP4Y = gtk_entry_get_text(g_setSurveyP4Y);
		surveyValueP4Z = gtk_entry_get_text(g_setSurveyP4Z);
		
		char *ptr;
		pulleyLocation[0].xLocation = strtof(surveyValueP1X,&ptr)*100.0;
		pulleyLocation[0].yLocation = strtof(surveyValueP1Y,&ptr)*100.0;
		pulleyLocation[0].zLocation = strtof(surveyValueP1Z,&ptr)*100.0;
		pulleyLocation[1].xLocation = strtof(surveyValueP2X,&ptr)*100.0;
		pulleyLocation[1].yLocation = strtof(surveyValueP2Y,&ptr)*100.0;
		pulleyLocation[1].zLocation = strtof(surveyValueP2Z,&ptr)*100.0;
		pulleyLocation[2].xLocation = strtof(surveyValueP3X,&ptr)*100.0;
		pulleyLocation[2].yLocation = strtof(surveyValueP3Y,&ptr)*100.0;
		pulleyLocation[2].zLocation = strtof(surveyValueP3Z,&ptr)*100.0;
		pulleyLocation[3].xLocation = strtof(surveyValueP4X,&ptr)*100.0;
		pulleyLocation[3].yLocation = strtof(surveyValueP4Y,&ptr)*100.0;
		pulleyLocation[3].zLocation = strtof(surveyValueP4Z,&ptr)*100.0;
		
		g_key_file_set_double (gkfd,"survey","surveyP1X",pulleyLocation[0].xLocation);
		g_key_file_set_double (gkfd,"survey","surveyP1Y",pulleyLocation[0].yLocation);
		g_key_file_set_double (gkfd,"survey","surveyP1Z",pulleyLocation[0].zLocation);
		g_key_file_set_double (gkfd,"survey","surveyP2X",pulleyLocation[1].xLocation);
		g_key_file_set_double (gkfd,"survey","surveyP2Y",pulleyLocation[1].yLocation);
		g_key_file_set_double (gkfd,"survey","surveyP2Z",pulleyLocation[1].zLocation);
		g_key_file_set_double (gkfd,"survey","surveyP3X",pulleyLocation[2].xLocation);
		g_key_file_set_double (gkfd,"survey","surveyP3Y",pulleyLocation[2].yLocation);
		g_key_file_set_double (gkfd,"survey","surveyP3Z",pulleyLocation[2].zLocation);
		g_key_file_set_double (gkfd,"survey","surveyP4X",pulleyLocation[3].xLocation);
		g_key_file_set_double (gkfd,"survey","surveyP4Y",pulleyLocation[3].yLocation);
		g_key_file_set_double (gkfd,"survey","surveyP4Z",pulleyLocation[3].zLocation);
		
		g_key_file_save_to_file(gkfd,CONFIG_FILE_NAME,NULL);

	}
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                PROGRAM EXIT
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void on_btnExit_clicked ()
{
	gtk_dialog_run(GTK_DIALOG(g_dlgExit));
	gtk_widget_hide(g_dlgExit);
}

void on_dlgExit_response (GtkDialog *dialog, gint response_id, gpointer user_data)
{
	//printf("\nNow response is %d\n", response_id);
	if (response_id == EXIT_YES)
	{
		//printf("saving current position\n");
		//save current position
		g_key_file_set_double (gkfd,"savedPosition","savedPositionX",xPositionCurrent);
		g_key_file_set_double (gkfd,"savedPosition","savedPositionY",yPositionCurrent);
		g_key_file_set_double (gkfd,"savedPosition","savedPositionZ",zPositionCurrent);
		
		g_key_file_save_to_file(gkfd,CONFIG_FILE_NAME,NULL);
		
		//free key file descriptor
		g_key_file_free (gkfd);

		gtk_main_quit();
	}
}

void on_window_main_destroy()
{
  gtk_main_quit();
}
