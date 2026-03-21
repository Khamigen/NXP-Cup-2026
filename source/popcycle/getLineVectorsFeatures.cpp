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

	if (finishDetected != NULL)
		*finishDetected = false;

	lv.noValidVectors = false;

	const float X_MIN = pixy.frameWidth * 0.25f;
	const float X_MAX = pixy.frameWidth * 0.75f;

	int finishScore = 0;   // ✅ 移到外面

	for (int i = 0; i < pixy.line.numVectors; i++)
	{
		Vector vi = pixy.line.vectors[i];

		float dx_i = vi.m_x1 - vi.m_x0;
		float dy_i = vi.m_y1 - vi.m_y0;
		float len_i = sqrtf(dx_i*dx_i + dy_i*dy_i);
		float xi = (vi.m_x0 + vi.m_x1) * 0.5f;

		// ROI + 基本條件
		if (xi < X_MIN || xi > X_MAX)
			continue;

		if (fabsf(dy_i) > 8 || len_i > 40 || vi.m_y0 < 30)
			continue;

		for (int j = i + 1; j < pixy.line.numVectors; j++)
		{
			Vector vj = pixy.line.vectors[j];

			float dx_j = vj.m_x1 - vj.m_x0;
			float dy_j = vj.m_y1 - vj.m_y0;
			float len_j = sqrtf(dx_j*dx_j + dy_j*dy_j);
			float xj = (vj.m_x0 + vj.m_x1) * 0.5f;

			if (xj < X_MIN || xj > X_MAX)
				continue;

			if (fabsf(dy_j) > 8 || len_j > 40 || vj.m_y0 < 30)
				continue;

			// scoring
			if (fabsf(vi.m_y0 - vj.m_y0) < 12)
				finishScore += 2;

			if (fabsf(len_i - len_j) < 15)
				finishScore += 2;

			if (fabsf(xi - xj) > 20)
				finishScore += 3;

			if (vi.m_y0 > 45 && vj.m_y0 > 45)
				finishScore += 2;
		}
	}

	if (finishDetected != NULL)
		*finishDetected = (finishScore >= 4);

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

void getLineVectorsFeature(Pixy2SPI_SS &pixy, LineVectors &lv){
	pixy.line.getAllFeatures(LINE_VECTOR, 1);
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
}
