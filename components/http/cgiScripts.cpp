//#pragma GCC optimize ( "0")

/*
 * cgiScripts.c
 *
 *  Created on: 8 nov. 2010
 *      Author: d.kleppe
 *      changes:
 */

//http://192.168.2.63/cgi-bin/Readvar?allSettings
#include <stdio.h>
#include <string.h>

#include "lwip/mem.h"
#include "cgiScripts.h"

#include "../../main/include/settings.h"
#include "../../main/include/sensorTask.h"
#include "../http/include/httpd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


extern int myRssi;

const tCGI *g_pCGIs;
int g_iNumCGIs;
extern bool settingsChanged;

#define NUM_CGIurls 12

// http:/192.168.2.7///cgi-bin/getLogMeasValues

const char* startCGIscript(int iIndex, char *pcParam);
const char* readCGIvalues(int iIndex, char *pcParam);

int readVarScript(char *pBuffer, int count);
extern int getLogScript(char *pBuffer, int count);
extern int getRTMeasValuesScript(char *pBuffer, int count);

int actionRespScript(char *pBuffer, int count);
bool readActionScript(char *pcParam);

int scriptState;
CGIresponseFileHandler_t readResponseFile;
int todoIndex;

const static char http_html_hdr[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n,";

typedef struct {
	const char *name;
	void *pValue;
	varType_t type;
	int nrValues;
} CGIdesc_t;

int readDescriptors(char *pBuffer, int count);

// do not alter
static const tCGI CGIurls[NUM_CGIurls] = {
//		{ "/cgi-bin/Readvar", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) readVarScript },  // !!!!!! index  !!
//		{ "/action_page.php", (tCGIHandler_t) readCGIvalues,(CGIresponseFileHandler_t) actionRespScript },
		{ "/cgi-bin/getLogMeasValues", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) getLogScript},
		{ "/cgi-bin/getRTMeasValues", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) getRTMeasValuesScript},
		{ "/cgi-bin/getSensorName", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) getSensorNameScript},
		{ "/cgi-bin/enableAutCal", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) enableAutCalScript},
		{ "/cgi-bin/disableAutCal", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) disableAutCalScript},
		{ "/cgi-bin/clearLog", (tCGIHandler_t) readCGIvalues, (CGIresponseFileHandler_t) clearLogScript},
};

static const CGIdesc_t CGIdescriptors[] = {
	//	{ "measValues", (void*) &measValues, STR, sizeof(measValues) / sizeof(char*) },
};


static const CGIdesc_t actionDescriptors[] = {

		//{ "calValue",&calValue, CALVAL, 1 }
};

const char* startCGIscript(int iIndex, char *pcParam) {
	int n;
	scriptState = 0;
	readResponseFile = CGIurls[iIndex].responseFileHandler;
	switch (iIndex) {
	case 0: // readvar
		for (n = 0; n < sizeof(CGIdescriptors) / sizeof(CGIdesc_t); n++) {
			if (strcmp(pcParam, CGIdescriptors[n].name) == 0) {
				todoIndex = n;
				break;
			}
		}
		if (n >= sizeof(CGIdescriptors) / sizeof(CGIdesc_t))
			return "";
		break;
	case 2:  // action script
		readActionScript(pcParam);
		return ("/spiffs/dmm.html");
		break;

	default:
		todoIndex = iIndex;
	}
	return ("/CGIreturn.txt");
}

bool readActionScript(char *pcParam) {
	int n, m, len;
	bool success = false;

	char *p = pcParam; // var=1.23&var2=4.56.	<tr>
	char *pDest, *pSrc;
	if (pcParam == NULL)
		return false;

	printf("%s\n", pcParam);
	char name[20];
	do {
		success = false;
		len = strlen(p);
		for (n = 0; n < len; n++) {
			if (p[n] == '=') {
				strncpy(name, p, n);
				name[n] = 0;
				for (m = 0; m < sizeof(actionDescriptors) / sizeof(CGIdesc_t); m++) {
					if (strcmp(name, actionDescriptors[m].name) == 0) { // found
						if (p[n + 1] != '&') { // empty value
							switch (actionDescriptors[m].type) {
							case FLT:
								sscanf(&p[n + 1], "%f", (float*) actionDescriptors[m].pValue); // read value
								break;
							case INT:
								sscanf(&p[n + 1], "%d", (int*) actionDescriptors[m].pValue); // read value
								break;
							case STR:
								pDest = (char*) actionDescriptors[m].pValue;
								pSrc = &p[n + 1];
								for (int m = 0; m < MAX_STRLEN - 1; m++) {
									if (*pSrc == '+') // spaces are replaced by '+' in HTML form
										*pSrc = ' ';
									*pDest++ = *pSrc++;
									if ((*pSrc == '&') || (*pSrc == 0)) // read until '&' or EOS
										break;

								}
								*pDest = 0;
								break;

							case CALVAL:
//								if (sscanf(&p[n + 1], "%lf", (double*) actionDescriptors[m].pValue) ==1) // read value
//									newCalValueReceived = true;
							break;

							case DESCR:
								break;
							}
						}
						success = true;
						break;
					}
				}
				break;
			}
		}
		p += n + 1;
		success = false;  // try to find next value
		for (n = 0; n < strlen(p); n++) {
			if (p[n] == '&') {
				p += n + 1;
				success = true;
				break;
			}
		}
	} while (success);

	settingsChanged = true;
	return false;
}

/**
 * parses CGI values
 * @param[in] iIndex not used
 * @param[in] pcParam pointer to string containing string with values
 * @param[out] pointer to responsefilename, this file is used to supply variable data to client
 */

const char* readCGIvalues(int iIndex, char *pcParam) {
	return startCGIscript(iIndex, pcParam);
}

// nothing		cnt = 0;
int actionRespScript(char *pBuffer, int count) {
//	char *pntr = pBuffer;
	int nrChars = 0;
	switch (scriptState) {
	case 0:
		// Open for reading hello.tx
		nrChars = sprintf(pBuffer,"%s",http_html_hdr);
		FILE* f = fopen("/spiffs/submRespOk.html", "r");  // fits in 8k
		if (f == NULL) {
			printf("submRespOk.html no found");
		}
		else {
			// 	printf(" hello found");

			nrChars += fread (pBuffer+nrChars,1,8000 ,f);
			fclose(f);
		}

		//	nrChars = sprintf(pntr, "%s", http_html_hdr);
		//	pntr += nrChars;
		//	nrChars += sprintf(pntr, "Ok\n");
		scriptState++;
		break;
	}
	return nrChars;
}

int readDescriptorsScript(char *pBuffer, int count) {
	switch (scriptState) {
	case 0:
		scriptState++;
		return readDescriptors(pBuffer, count);

		break;
	default:
		break;
	}
	return 0;
}

int doCalibrateScript(char *pBuffer, int count) {
switch (scriptState) {
case 0:
	scriptState++;
	strcpy (pBuffer,"Ok");
	return 3;

	break;
default:
	break;
}
return 0;
}



int readVarScript(char *pBuffer, int count) {
	char *pntr = pBuffer;
	int nrChars = 0;
	int cnt = 0;
//	void *pValue;
	uint8_t * pValue;
	int nrValues = 1;
	settingsDescr_t *pDescr;
	int n;

	//pValue = (float*) CGIdescriptors[todoIndex].pValue;
	pValue = (uint8_t*) CGIdescriptors[todoIndex].pValue;
	nrValues = CGIdescriptors[todoIndex].nrValues;

	switch (scriptState) {
	case 0:
		nrChars = sprintf(pntr, "%s", http_html_hdr);
		pntr += nrChars;
		if (CGIdescriptors[todoIndex].type == DESCR) {
			pDescr = (settingsDescr_t*) CGIdescriptors[todoIndex].pValue;
			while (pDescr->size > 0) {
				pValue = (uint8_t *) pDescr->pValue;
				for (n = 0; n < pDescr->size; n++) {
					switch (pDescr->varType) {
					case FLT:
						cnt = sprintf(pntr, "%2.1f,", *(float*) pValue);
						pValue += sizeof(float);
						break;
					case INT:
						cnt = sprintf(pntr, "%d,", *(int*) pValue);
						pValue += sizeof(int);
						break;
					case STR:
						cnt = sprintf(pntr, "%s,", (char*) pValue);
						pValue += MAX_STRLEN + 1;
						break;
					case DESCR:
					case CALVAL:
						break;

					}
					pntr += cnt;
					nrChars += cnt;
				}
				pDescr++;
			}
		} else {
			for (int n = 0; n < nrValues; n++) {
				switch (CGIdescriptors[todoIndex].type) {
				case FLT:
					cnt = sprintf(pntr, "%2.1f", *(float*) pValue);
					pValue += sizeof(float);
					break;
				case INT:
					cnt = sprintf(pntr, "%d", *(int*) pValue);
					pValue += sizeof(int);
					break;
				case STR:
					cnt = sprintf(pntr, "%s", (char*) pValue);
					pValue += MAX_STRLEN + 1;
					break;
				case DESCR:
				case CALVAL:
					break;
				}
				pntr += cnt;
				if (n < (nrValues - 1)) {
					*pntr++ = ',';
					cnt++;
				}
				nrChars += cnt;
			}
		}
		scriptState++;
		break;
	}
	return nrChars;
}


void CGI_init(void) {
	g_pCGIs = CGIurls;
	g_iNumCGIs = NUM_CGIurls;

}

