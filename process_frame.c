/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define TEST_CALC_DERIV         1
#define TEST_AVG_DERIV          0
#define TEST_CORNER_INDEX       0
#define TEST_MAX_CORNER_INDEX   1

#define IMG_SIZE    NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

#define FILTER GaussFilter
#define BORDER (sizeof(FILTER) - 1)

#define DEFAULT_K 5

#define BOXSIZE 5

void CalcDeriv(void);
void AvgDeriv(int Index);
void CalcCornerIndex(int k);
void MaxCornerIndex(void);
void MarkCorner(void);

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;
const uint8 GaussFilter[] = {82, 72, 50, 27, 11, 4, 1};
const uint8 BinFilter[] = {128, 64, 64, 32, 8, 4, 1};

int TextColor;
int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];
int32_t Mc[2][IMG_SIZE];

void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}


void ProcessFrame()
{
	uint32 t1, t2;
	char Text[] = "hallo world";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		//t1 = OscSupCycGet();
		//example for copying sensor image to background image
	//memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		//example for time measurement
		//t2 = OscSupCycGet();

		//example for log output to console
		//OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));

		//example for drawing output
		//draw line
		//DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		//DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}

    t1 = OscSupCycGet();

    CalcDeriv(); /* calculate derivatives Ix^2, Iy^2 and Ix*Iy */
    AvgDeriv(0); /* average derivative Ix^2 */
    AvgDeriv(1); /* average derivative Iy^2 */
    AvgDeriv(2); /* average derivative I{xy} */
    CalcCornerIndex(DEFAULT_K); /* Calculate "Corner Index" */
    MaxCornerIndex();   /* local maximas of "Corner Index" */
    MarkCorner(); /* Mark corners */

    t2 = OscSupCycGet();
    OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));
}

/*! \fn void CalcDeriv(void)
 *  \brief Calculate derivatives {I_x}^2, {I_x}^2 and I_x \cdot I_y
 *
 *  \return void
 *
 *  Global input variables:
 *      data.u8TempImage[SENSORIMG]: Input image
 *  Global output variables:
 *      avgDxy[0]: {I_x}^2
 *      avgDxy[1]: {I_y}^2
 *      avgDxy[2]: I_x \cdot I_y
 */
void CalcDeriv(void)
{
    int c, r;

    for(r = nc; r < nr*nc-nc; r+= nc) {/* we skip the first and last line */
        for(c = 1; c < nc-1; c++) {
            /* do pointer arithmetics with respect to center pixel location */
            unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];

            /* implement Sobel filter */
            int dx = -(int) *(p-nc-1) + (int) *(p-nc+1)
                -2* (int) *(p-1) + 2* (int) *(p+1)
                -(int) *(p+nc-1) + (int) *(p+nc+1);
            int dy = -(int) *(p-nc-1) - 2 * (int) *(p-nc) - (int) *(p-nc+1) +
                    (int) *(p+nc-1) + 2 * (int) *(p+nc) + (int) *(p+nc+1);

            avgDxy[0][r+c] = dx*dx;
            avgDxy[1][r+c] = dy*dy;
            avgDxy[2][r+c] = dx*dy;

            #if TEST_CALC_DERIV
                //data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, 128 + (avgDxy[0][r+c]>>10)));
                //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[1][r+c]>>10)));
                data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, 128 + (avgDxy[0][r+c]>>7)));
                data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[2][r+c]>>10)));
            #endif /* TEST_CALC_DERIV */
        }
    }
}

/*! \fn void AvgDeriv(int Index)
 *  \brief Calculate average of derivatives {I_x}^2, {I_x}^2 and I_x \cdot I_y
 *
 *  \param  Index Derivative to be averaged
 *  \return void
 *
 *  Global input variables:
 *      data.u8TempImage[SENSORIMG]: Input image
 *  Global output variables:
 *      avgDxy[0]: {I_x}^2
 *      avgDxy[1]: {I_y}^2
 *      avgDxy[2]: I_x \cdot I_y
 */
void AvgDeriv(int Index) {
    int c, r;

    /* x */
    for (r = nc; r < nr*nc-nc; r += nc) {
        for (c = (BORDER + 1); c < nc - (BORDER + 1); c++) {
            int* p = &avgDxy[Index][r+c];
            int sx = (*p)*GaussFilter[0];
            for (int i = 1; i < sizeof(FILTER); i++) {
                sx += ((*(p - i)) + (*(p + i))) * FILTER[i];
            }
            helpBuf[r+c] = (sx >> 8);
        }
    }

    /* y */
    for (r = nc; r < nr*nc-nc; r += nc) {
        for (c = (BORDER + 1); c < nc - (BORDER + 1); c++) {
            int* p = &helpBuf[r+c];
            int sy = (*p)*FILTER[0];
            for (int i = 1; i < sizeof(FILTER); i++) {
                sy += ((*(p - (i * nc))) + (*(p + (i * nc)))) * FILTER[i];
            }
            avgDxy[Index][r+c] = (sy >> 8);

            #if TEST_AVG_DERIV
                //if (Index == 0) {
                //    data.u8TempImage[BACKGROUND][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>10)));
                //    //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>7)));
                //}
                //if (Index == 2) {
                //    data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>10)));
                //    //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>7)));
                //}
                if (Index == (TEST_AVG_DERIV - 1)) {
                    data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>10)));
                    //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (avgDxy[Index][r+c]>>7)));
                }
            #endif /* TEST_AVG_DERIV */
        }
    }
}

/*! \fn void CalcCornerIndex(int k)
 *  \brief Calculate "Corner Index"
 *
 *  \f[
 *      M_c = \left(\left({I_x}^2\right) \cdot \left({I_y}^2\right) 
 *          - \left(I_x \cdot I_y\right)^2\right)
 *          - k \cdot \left(\left({I_x}^2\right) + \left({I_y}^2\right)\right)^2
 *  \f]
 *
 *  \param  k Parameter k in formula above
 *  \return void
 *
 *  Global input variables:
 *      avgDxy[0]: {I_x}^2
 *      avgDxy[1]: {I_y}^2
 *      avgDxy[2]: I_x \cdot I_y
 *  Global output variables:
 *      Mc[0]: M_c
 */
void CalcCornerIndex(int k) {
    int c, r;
    int32_t Ix2, Iy2, Ixy;

    for (r = nc * (BORDER+1); r < (nr * nc) - (nc * (BORDER+1)); r += nc) {
        for (c = (BORDER + 1); c < nc - (BORDER + 1); c++) {
            Ix2 = avgDxy[0][r+c] >> 7;
            Iy2 = avgDxy[1][r+c] >> 7;
            Ixy = avgDxy[2][r+c] >> 7;
            Mc[0][r+c] = (Ix2 * Iy2 - Ixy * Ixy) - ((5 * (Ix2 + Iy2) * (Ix2 + Iy2))>>7);

            #if TEST_CORNER_INDEX
                data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[0][r+c]>>5)));
                //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[0][r+c]>>7)));
            #endif /* TEST_CORNER_INDEX */
        }
    }
}

/*! \fn void MaxCornerIndex(void)
 *  \brief Calculate local maximas of "Corner Index"
 *
 *  \return void
 *
 *  Global input variables:
 *      Mc[0]: M_c
 *  Global output variables:
 *      Mc[1]: max(M_c)
 */
void MaxCornerIndex(void) {
    int c, r, ct, rt;

    for (r = nc * (BORDER+1); r < (nr * nc) - (nc * (BORDER+1)); r += nc) {
        for (c = (BORDER + 1); c < nc - (BORDER + 1); c++) {
            Mc[1][r+c] = Mc[0][r+c];
            for (rt = -6; rt < 7; rt++) {
                for (ct = -6 * nc; ct < 7 * nc; ct+=nc) {
                    if (Mc[0][r+c+rt+ct] > Mc[0][r+c]) {
                        Mc[1][r+c] = 0;
                        rt = 7;
                        ct = 7 * nc;
                    }
                }
            }

            #if TEST_MAX_CORNER_INDEX
                data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[1][r+c]>>7)));
                //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[1][r+c]>>7)));
            #endif /* TEST_MAX_CORNER_INDEX */
        }
    }
}

/*! \fn void MarkCorner()
 *  \brief Calculate local maximas of "Corner Index"
 *
 *  \return void
 *
 *  Global input variables:
 *      Mc[1]: max(M_c)
 *  Global output variables:
 */
void MarkCorner() {
    int c, r, count, threshold;
    threshold = ((255*data.ipc.state.nThreshold)/100);
    if (threshold <= 10) {
        threshold = 255;
    }

    count = 0;
    for (r = nc * (BORDER+1); r < (nr * nc) - (nc * (BORDER+1)); r += nc) {
        for (c = (BORDER + 1); c < nc - (BORDER + 1); c++) {
            if (Mc[1][r+c] >= threshold) {
                count++;
                DrawBoundingBox(c-BOXSIZE,
                        (r/nc)-BOXSIZE,
                        c+BOXSIZE,
                        (r/nc)+BOXSIZE,
                        false,
                        RED);
            }

            #if TEST_MAX_CORNER_INDEX
                data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[1][r+c]>>7)));
                //data.u8TempImage[THRESHOLD][r+c]  = (uint8) MIN(255, MAX(0, 128 + (Mc[1][r+c]>>7)));
            #endif /* TEST_MAX_CORNER_INDEX */
        }
    }
    printf("Number of points: %d ", count);
    printf("Slider value: %d ", threshold);
}
