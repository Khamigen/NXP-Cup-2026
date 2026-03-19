/*
 * getLineVectorsFeature.cpp
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */

#include <Pixy/Pixy2SPI_SS.h>
#include <Popcycle/getLineVectorsFeature.h>
#include <stdbool.h>
#include <math.h>

void getLineVectorsFeature(Pixy2SPI_SS &pixy, LineVectors &lv, bool *finishDetected){
	pixy.line.getAllFeatures(LINE_VECTOR, 1);

	// reset
	if (finishDetected != NULL)
		*finishDetected = false;

	lv.noValidVectors = false;

	// 🛑 Step 1: 強化版 finish detection
	for (int i = 0; i < pixy.line.numVectors; i++)
	{
		Vector vi = pixy.line.vectors[i];

		float dx_i = vi.m_x1 - vi.m_x0;
		float dy_i = vi.m_y1 - vi.m_y0;
		float len_i = sqrtf(dx_i*dx_i + dy_i*dy_i);
		float xi = (vi.m_x0 + vi.m_x1) * 0.5f;

		// 基本條件：水平 + 短 + 在畫面下半部
		if (fabsf(dy_i) > 5 || len_i > 30 || vi.m_y0 < 40)
			continue;

		for (int j = i + 1; j < pixy.line.numVectors; j++)
		{
			Vector vj = pixy.line.vectors[j];

			float dx_j = vj.m_x1 - vj.m_x0;
			float dy_j = vj.m_y1 - vj.m_y0;
			float len_j = sqrtf(dx_j*dx_j + dy_j*dy_j);
			float xj = (vj.m_x0 + vj.m_x1) * 0.5f;

			if (fabsf(dy_j) > 5 || len_j > 30 || vj.m_y0 < 40)
				continue;

			// 🔥 強化條件（避免誤判）
			if (fabsf(vi.m_y0 - vj.m_y0) < 10 &&     // 同一水平
				fabsf(len_i - len_j) < 10 &&         // 長度接近
				fabsf(xi - xj) > 20)                // 有間距（關鍵）
			{
				if (finishDetected != NULL)
					*finishDetected = true;
				break;
			}
		}

		if (finishDetected != NULL && *finishDetected)
			break;
	}

	if (pixy.line.numVectors >= 2){
		lv.useSingleVectorLogic = false;
		lv.v1 = pixy.line.vectors[0];
		lv.v2 = pixy.line.vectors[1];
	} else if(pixy.line.numVectors == 1) {
		lv.useSingleVectorLogic = true;
		lv.v1 = pixy.line.vectors[0];
	} else {
		lv.noValidVectors = true;
	};
};

