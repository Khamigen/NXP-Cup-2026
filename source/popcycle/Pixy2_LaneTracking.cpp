/*
 * Pixy2_Lane_Tracking.c
 *
 *  Created on: 29 Nov 2025
 *      Author: j6895
 */
#include <Pixy/Pixy2SPI_SS.h>
#include <Popcycle/Pixy2_LaneTracking.h>
#include <stdbool.h>
#include <math.h>
#include <algorithm>
extern "C"{
#include "Modules/mTimer.h"
}

#define MA_WINDOW_SIZE 3 // window used for moving average

//static because these are "state" saved from last loop. shouldn't be reset during each loop.
// moving average
static float errorBuffer[MA_WINDOW_SIZE];
static int bufferIndex = 0;
static int bufferCount = 0;

// Proportional–Derivative Controller
static float lastAvgError = 0.0f;
const float kD = 0.01f;	//derivative, bigger kd, faster steer
const float kP = -0.04f;	//proportion, bigger kp, bigger steer

// Limit maximum steer
const float steerMax = 0.75f;

// Limit steering rate
static float lastSteer = 0.0f;
const float steerStepLimit = 0.5f;

// missed Vector
static int lastLaneCenterX = 39;
//static int lastHadTwoLines = 0;

const int frameCenterX = 39; // Pixy2 line mode width/2
const int frameBottomDeadzoneY = 35; // Pixy line mode bottom Y = 51
const int laneHalfWidthPx = 25;   //
const int jumpThreshold = 25;     // 若新估跳超過此值則暫不採用
int singleLineStableCount = 0;    // 單線穩定計數器
const int stabilityFrames = 3;    // 要連續多少幀才接受估值

const int MIN_TWO_HEIGHT = 18;      // 18~22 可調
const int MIN_TWO_BOTTOM_Y = 40;    // 38~42 可調
const int MIN_SINGLE_HEIGHT = 18;   // 可微調 16~22
const int MIN_SINGLE_BOTTOM_Y = 38;   // 可微調 36~40
static int intersectionHold = 0;
static const int INTERSECTION_HOLD_FRAMES = 6; // 34ms * 10 ≈ 340ms，可調 8~12
//angle calculation
static inline float angleDegPixy(const Vector &v)
{
    float angle = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0) * 57.2958f;
    if (v.m_x1 < v.m_x0) angle = -angle; // 保留你既修正
    return angle;
}

// intersection detection
static bool isIntersection(const Pixy2SPI_SS &pixy)
{
    const int centerL = frameCenterX - 6;
    const int centerR = frameCenterX + 6;

    int inCenter = 0;
    int nearH = 0;       // 近水平
    int nearV = 0;       // 近垂直
    int farCnt = 0;
    int n = pixy.line.numVectors;
    const int FAR_BOTTOM_Y = 38;
    // 你畫面高 51，呢個係「比較接近車」的門檻
    for (int i = 0; i < pixy.line.numVectors; i++)
    {
        const auto &v = pixy.line.vectors[i];
        int midX = (v.m_x0 + v.m_x1) / 2;
        int bottomY = std::max(v.m_y0, v.m_y1);
        float a = angleDegPixy(v);

        if (midX >= centerL && midX <= centerR) inCenter++;

        // 角度：相對 x 軸
        // 水平 ~ 0 度
        if (fabsf(a) < 12.0f) nearH++;
        // 垂直 ~ +/-90 度
        if (fabsf(fabsf(a) - 90.0f) < 18.0f) nearV++;

        // 遠：底部唔貼近畫面底（你畫面高 51）
        if (bottomY < 38) farCnt++;
    }

    // --- 核心判斷 ---
        // 交叉口通常：同時出現「近水平」+「近垂直」，
        // 而且中心帶冇明確車道線（inCenter==0）會加強可信度，
        // 同時 farCnt 多，代表睇到嘅係遠處交叉線/角落線。
        bool looksLikeIntersection =
            (n >= 3) &&
            (nearH >= 1) &&
            (nearV >= 1) &&
            (inCenter == 0) &&
            (farCnt >= 2);

        // --- 連續幀確認（去抖）---
        static int stable = 0;
        if (looksLikeIntersection) stable++;
        else stable = 0;

        // 連續 2 幀先算（你 main loop 34ms，2 幀 ~ 68ms）
        return (stable >= 2);
}
//steer function for intersection
static int intersectionLogic()
{
    // ✅ 用上一幀 laneCenterX 作為基礎，逐步拉返去畫面中心
    // pull 越大 → 越快回正，但可能在大角度進入交叉口時出事
    const float pull = 0.15f;  // 可調 0.08 ~ 0.25

    float target = (float)lastLaneCenterX + ((float)frameCenterX - (float)lastLaneCenterX) * pull;

    // 轉成 int
    return (int)(target + 0.5f);
}
//check whether the 2 vectors from pixy are valid
bool twoVectorsValid(const Vector &v1, const Vector &v2)
{
	// angle calculation
	float angle1 = atan2f(v1.m_y1 - v1.m_y0, v1.m_x1 - v1.m_x0) * 57.2958f;
	float angle2 = atan2f(v2.m_y1 - v2.m_y0, v2.m_x1 - v2.m_x0) * 57.2958f;
	float angleDiff = fabsf(angle1 - angle2);
	// calculate centerX of vectors
	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
	// calculate bottomY of vectors
	int bottomY1 = std::max(v1.m_y0,v1.m_y1);
	int bottomY2 = std::max(v2.m_y0,v2.m_y1);
	// vector height
	int height1 = abs(v1.m_y1 - v1.m_y0);
	int height2 = abs(v2.m_y1 - v2.m_y0);
	// check if vectors be on each side of lane
	bool bothRight = (midX1 > frameCenterX && midX2 > frameCenterX);
	bool bothLeft = (midX1 < frameCenterX && midX2 < frameCenterX);
	//condition 1: angle difference too big
	if (angleDiff > 50.0f)
		return false;
	//condition 2: 2 vectors too close to each other
	if (abs(midX1 - midX2) < 40)
		return false;
	//condition 3: depth difference too big
	if (abs(bottomY1 - bottomY2) > 25)
		return false;
	//condition 4: both vectors on same side of lane
	if (bothRight || bothLeft)
		return false;
	// ===== 新增：交叉口角落剔除 =====
	if (height1 < MIN_TWO_HEIGHT || height2 < MIN_TWO_HEIGHT) return false;
    if (std::max(bottomY1, bottomY2) < MIN_TWO_BOTTOM_Y) return false;
	return true;
}
float singleVectorConfidence(const Vector &v)
{
    float confidence = 1.0f;

    int height = abs(v.m_y1 - v.m_y0);
    int bottomY = std::max(v.m_y0, v.m_y1);
    float len = hypotf(v.m_x1 - v.m_x0, v.m_y1 - v.m_y0);

    // 距離車太遠 → 不可信
    if (bottomY < MIN_SINGLE_BOTTOM_Y)
        confidence *= 0.2f;

    // 太短 → 容易誤判方向
    if (height < MIN_SINGLE_HEIGHT)
        confidence *= 0.4f;

    // 太短的向量
    if (len < 20.0f)
        confidence *= 0.5f;

    // clamp
    if (confidence < 0.0f) confidence = 0.0f;
    if (confidence > 1.0f) confidence = 1.0f;

    return confidence;
}
static float scorePair(const Vector &a, const Vector &b)
{
    float a1 = atan2f(a.m_y1 - a.m_y0, a.m_x1 - a.m_x0) * 57.2958f;
    float a2 = atan2f(b.m_y1 - b.m_y0, b.m_x1 - b.m_x0) * 57.2958f;

    float angleDiff = fabsf(a1 - a2);

    int midX1 = (a.m_x0 + a.m_x1) / 2;
    int midX2 = (b.m_x0 + b.m_x1) / 2;
    int dxMid = abs(midX1 - midX2);

    int bottomY1 = std::max(a.m_y0, a.m_y1);
    int bottomY2 = std::max(b.m_y0, b.m_y1);
    int dyBottom = abs(bottomY1 - bottomY2);

    bool bothRight = (midX1 > frameCenterX && midX2 > frameCenterX);
    bool bothLeft  = (midX1 < frameCenterX && midX2 < frameCenterX);

    // hard reject（跟你 twoVectorsValid 類似）
    if (angleDiff > 55.0f) return -1e9f;
    if (dxMid < 35)        return -1e9f;
    if (dyBottom > 25)     return -1e9f;
    if (bothRight || bothLeft) return -1e9f;

    // soft score：越大越好
    float score = 0.0f;

    // 兩條越接近「平行」越好（angleDiff 越小越好）
    score += (55.0f - angleDiff) * 2.0f;

    // 間距越像車道寬越好：你可以用期望值（例如 45~65）做加分
    // 這裡做簡單：距離越大越好，但太大也扣分
    score += dxMid;
    if (dxMid > 70) score -= (dxMid - 70) * 3.0f;

    // 越靠近車（bottomY 越大）越好：避免選遠處線 / 角落
    score += bottomY1 + bottomY2;

    // 角落組合常出現「一條很水平、一條很垂直」
    // 你可以把接近水平的 pair 扣分（視你賽道而定）
    bool nearHorizontalPair = (fabsf(a1) < 18.0f) || (fabsf(a2) < 18.0f);
    if (nearHorizontalPair) score -= 40.0f;

    return score;
}


bool singleVectorValid(const Vector &v)
{
	float dx = v.m_x1 - v.m_x0;
	float dy = v.m_y1 - v.m_y0;
	//angle calculation
	float angle = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0) * 57.2958f;
	if ( v.m_x1 < v.m_x0)
	{
		angle = -angle;
	}
	//
	//middle X coordinate
	int midX = (v.m_x0 + v.m_x1) / 2;
    //angle = 0 --> horizontal line
	bool rightTurn = (angle > 25);   // vector points rightward
    bool leftTurn  = (angle < -25);  // vector points leftward
	//length calculation
	float len = hypotf(dx,dy);
	int height = abs(v.m_y1 - v.m_y0);
	int bottomY = std::max(v.m_y0, v.m_y1);

	if (height < MIN_SINGLE_HEIGHT)
	    return false;

	if (bottomY < MIN_SINGLE_BOTTOM_Y)
	    return false;
	// intersection pattern
//	bool nearHorizontal = fabsf(angle)<20.0f;
//	bool commonLengthInIntersection = (len > 20.0f && len < 50.0f);
//	bool doesNotTouchBottom = std::max(v.m_y0,v.m_y1)< 70;
//	bool outerLinePositionFalse = (rightTurn && midX > frameCenterX)||(leftTurn && midX < frameCenterX);
//	bool tooShortVertically = (height < MIN_SINGLE_HEIGHT);
//	bool notCloseToCar = (bottomY < MIN_SINGLE_BOTTOM_Y);
//
//	if (nearHorizontal&&commonLengthInIntersection)
//		return false;
//	if (doesNotTouchBottom)
//		return false;
//	if (outerLinePositionFalse)
//		return false;
//	if (tooShortVertically)
//		return false;
//	if (notCloseToCar)
//		return false;
	return true;
}

//lane center calculation for single line case
static inline float turnBoost(const Vector &v)
{
	const float H = 51.0f; // Pixy line-mode height
	float dy = fabsf(v.m_y0-v.m_y1);     // 0..50
	float f  = dy / (H - 1.0f);        // 0..1
	f = std::clamp(f, 0.0f, 1.0f);        // 0(top) -> 1(bottom)
    return 1.0f + 0.7f * f;
}
float singleVectorLogic(Vector &v)
{

	float angle = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0) * 57.2958f;
	float slope = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0);

	//angle calculation
	if ( v.m_x1 < v.m_x0)
	{
		angle = -angle;
	}

    //middle X coordinate
    int midX = (v.m_x0 + v.m_x1) / 2;
    //angle = 0 --> horizontal line
    bool rightTurn = (angle > 25);   // vector points rightward
    bool leftTurn  = (angle < -25);  // vector points leftward
    int offset = (int)(laneHalfWidthPx * turnBoost(v));
    // case 1: right turn
    if(rightTurn)
        return midX - offset;

    // case 2: left turn
    if(leftTurn)
        return midX + offset;

    // case 3: angle close to 0, almost horizontal line, use the position of the line to calculate lane center
    // in this scenario, right outer line would be on the right side and vice versa

    if(slope < 0)
        return midX + offset;  // vector located at the right, turn right
    else
        return midX - offset;  // vector located at the left, turn left
}

float singleVectorSmooth(Vector &v, int laneCenterEstimate)
{
	// --- compute vector length ---
	float len = hypotf(v.m_x1 - v.m_x0, v.m_y1 - v.m_y0);
	const float minLen = 15.0f;  // minimal vector length to consider, adjust empirically
	const float normalLen = 60.0f; // typical full-length vector in pixels, adjust for your camera
	if(len < minLen)
		// too short, ignore it and return lastLaneCenter
		return lastLaneCenterX;
	// longer vector gets more weight
	float lengthWeight = (len < normalLen) ? (len / normalLen) : 1.0f;
	// calculate the Y coordinate of the vector, upper vector gets more weight
	int midY = (v.m_y0 + v.m_y1) / 2;
	const int frameHeight = 80;  // Pixy2 line-mode height
	float verticalWeight = 1.0f - ((float)midY / (float)frameHeight);  // 0 at bottom, 1 at top
	//combine length and vertical weight
	float combinedWeight = lengthWeight * verticalWeight;
	//call confidence
	float conf = singleVectorConfidence(v);   // 0..1
	// conf 低 → 更新更慢；conf 高 → 保持原本速度
	// 0.2f 是下限：避免 conf 太低時完全不動，造成卡住+跳變
    float weightWithConf = combinedWeight * (0.2f + 0.8f * conf);
	// --- blend with previous lane center ---
	int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * weightWithConf);
	// --- jump protection ---
	int delta = laneCenterEstimate - lastLaneCenterX;
	// conf 越低 → threshold 越小（更嚴格）
	// conf 越高 → threshold 越接近原本 jumpThreshold
	int dynJump = (int)(jumpThreshold * (0.3f + 0.7f * conf)); // 0.3~1.0 倍
	if (abs(delta) > dynJump)
	{
	    // small change
		blended = lastLaneCenterX + (int)(delta * 0.05f);
	}
	else
	{
		//change with confidence
		blended = lastLaneCenterX + (int)(delta * (0.2f + 0.6f * conf));
	}
	return blended;
}

float Pixy2_LaneTracking(Pixy2SPI_SS &pixy){
	bool inIntersectionMode = false;
	int laneCenterX;
	pixy.line.getAllFeatures(LINE_VECTOR, 1);

	//intersection hold
//	if (intersectionHold > 0)
//	{
//	    intersectionHold--;
//	    inIntersectionMode = true;
//
//	    laneCenterX = intersectionLogic();
//	}
//	else if (pixy.line.numVectors >= 2 && isIntersection(pixy))
//	{
//	    intersectionHold = INTERSECTION_HOLD_FRAMES;
//	    inIntersectionMode = true;
//
//	    laneCenterX = intersectionLogic();
//	}
//	//not intersection, detects more than 2 vectors
//	else if(pixy.line.numVectors >= 2)
//	    {
//			// ✅ 先揀 best pair
//			int bestI = -1, bestJ = -1;
//			float bestScore = -1e9f;
//			int n = pixy.line.numVectors;
//			if (n > 6) n = 6; // 可選：限制運算量
//
//			for (int i = 0; i < n; i++)
//			{
//				for (int j = i + 1; j < n; j++)
//				{
//					float s = scorePair(pixy.line.vectors[i], pixy.line.vectors[j]);
//					if (s > bestScore)
//					{
//						bestScore = s;
//						bestI = i;
//						bestJ = j;
//					}
//				}
//			}
//			if (bestI < 0)
//		    {
//		        laneCenterX = lastLaneCenterX;
//		    }
//		   else
//		    {
//			    // Determine left and right lines
//			   	auto v1 = pixy.line.vectors[0];
//			   	auto v2 = pixy.line.vectors[1];
//			    //check if the 2 vectors valid
//			    //if valid, calculate using 2 vector logic
//			    if (twoVectorsValid(v1,v2)){
//			    //2 vector logic
//		    	//calculate mid x coordinates
//		    	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
//		    	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
//		    	//compare midX1 and midX2, the smaller one is leftX and the bigger one is rightX
//		    	int leftX = (midX1 < midX2) ? midX1 : midX2;
//		    	int rightX = (midX1 < midX2) ? midX2 : midX1;
//
//		    	laneCenterX = (leftX + rightX)/2;
//		    	lastLaneCenterX = laneCenterX;
//		    }
//		    //if not valid, fallback to 1 vector logic
//		    else
//		    {
//		    	int laneCenterEstimate = singleVectorLogic(v1);
//		   		// smooth the vector
//		   		lastLaneCenterX = singleVectorSmooth(v1,laneCenterEstimate);
//				laneCenterX = lastLaneCenterX;
//	    	}
//	    }
//	}
	// only detects 1 vector
	if (pixy.line.numVectors == 1)
	{
	    auto v = pixy.line.vectors[0];

	    	// use single vector logic
	    	int laneCenterEstimate = singleVectorLogic(v);
		   	// smooth the vector
		   	lastLaneCenterX = singleVectorSmooth(v,laneCenterEstimate);
		   	laneCenterX = lastLaneCenterX;
	}
	//detects no vector
	else{
		//use last lane center
		laneCenterX = lastLaneCenterX;
	}

	// error calculation
	float error = (float)(laneCenterX - frameCenterX);
    // Moving Average
	//add error to the buffer array
    errorBuffer[bufferIndex] = error;
    //bufferIndex point to next element, use modulo to loop to element0 if window size is reached
    bufferIndex = (bufferIndex + 1) % MA_WINDOW_SIZE;

    if(bufferCount < MA_WINDOW_SIZE)
        {bufferCount++;}

    float sum = 0;
    for(int i=0; i<bufferCount; i++)
        {sum += errorBuffer[i];}

    float avgError = sum / bufferCount;

    //PD Controller, kP and kD defined as constant
    //float dError = avgError - lastAvgError;
    lastAvgError = avgError;

    float steer = kP * avgError;

    // Limit the maximum range of steer, steerMax defined as constant
    // to do: use clampf instead
    if(steer > steerMax) steer = steerMax;
    if(steer < -steerMax) steer = -steerMax;

    // Limit turn rate. steerStepLimit defined as constant
    float delta = steer - lastSteer;
    if(delta > steerStepLimit)
        {steer = lastSteer + steerStepLimit;}
    else if(delta < -steerStepLimit)
        {steer = lastSteer - steerStepLimit;}

    lastSteer = steer;

    return steer;
}

float Pixy2_LaneTrackingDebug(Pixy2SPI_SS &pixy, pixyLineVector (&vectorData)[2]){
	int laneCenterX;
	pixy.line.getAllFeatures(LINE_VECTOR, 1);
	// if detects more than 2 vectors, calculate the center
	if(pixy.line.numVectors >= 2)
	    {
	        // Determine left and right lines
	        auto v1 = pixy.line.vectors[0];
	        auto v2 = pixy.line.vectors[1];
	        //writing vector data to struct for sd card logging
	        vectorData[0].m_x0 = v1.m_x0;
	        vectorData[0].m_y0 = v1.m_y0;
	        vectorData[0].m_x1 = v1.m_x1;
	        vectorData[0].m_y1 = v1.m_y1;

	        vectorData[1].m_x0 = v2.m_x0;
	        vectorData[1].m_y0 = v2.m_y0;
	        vectorData[1].m_x1 = v2.m_x1;
	        vectorData[1].m_y1 = v2.m_y1;
	        //check if the 2 vectors valid
	        //if valid, calculate using 2 vector logic
		    if (twoVectorsValid(v1,v2)){
		    	//2 vector logic
		    	//calculate mid x coordinates
		    	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
		    	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
		    	//compare midX1 and midX2, the smaller one is leftX and the bigger one is rightX
		    	int leftX = (midX1 < midX2) ? midX1 : midX2;
		    	int rightX = (midX1 < midX2) ? midX2 : midX1;

		    	laneCenterX = (leftX + rightX)/2;
		    	lastLaneCenterX = laneCenterX;
		    }
		    //if not valid, fallback to 1 vector logic
		    else{
		    	// use single vector logic
		    	int laneCenterEstimate = singleVectorLogic(v1);
		    	// smooth the vector
		    	lastLaneCenterX = singleVectorSmooth(v1,laneCenterEstimate);
		    	laneCenterX = lastLaneCenterX;
		    }
	    }
	// only detects 1 vector
	else if (pixy.line.numVectors == 1)
	{
	    auto v = pixy.line.vectors[0];
	    // use single vector logic
	    int laneCenterEstimate = singleVectorLogic(v);
	    // smooth the vector
	    lastLaneCenterX = singleVectorSmooth(v,laneCenterEstimate);
	    laneCenterX = lastLaneCenterX;
	}
	//detects no vector
	else{
		// use last lane center
		laneCenterX = lastLaneCenterX;
	}

	// error calculation
	float error = (float)(laneCenterX - frameCenterX);
    // Moving Average
	//add error to the buffer array
    errorBuffer[bufferIndex] = error;
    //bufferIndex point to next element, use modulo to loop to element0 if window size is reached
    bufferIndex = (bufferIndex + 1) % MA_WINDOW_SIZE;

    if(bufferCount < MA_WINDOW_SIZE)
        {bufferCount++;}

    float sum = 0;
    for(int i=0; i<bufferCount; i++)
        {sum += errorBuffer[i];}

    float avgError = sum / bufferCount;

    //PD Controller, kP and kD defined as constant
    float dError = avgError - lastAvgError;
    lastAvgError = avgError;

    float steer = kP * avgError + kD * dError;

    // Limit the maximum range of steer, steerMax defined as constant
    // to do: use clampf instead
    if(steer > steerMax) steer = steerMax;
    if(steer < -steerMax) steer = -steerMax;

    // Limit turn rate. steerStepLimit defined as constant
    float delta = steer - lastSteer;
    if(delta > steerStepLimit)
        {steer = lastSteer + steerStepLimit;}
    else if(delta < -steerStepLimit)
        {steer = lastSteer - steerStepLimit;}

    lastSteer = steer;

    return steer;
}

//float Pixy2_LaneTrackingDebug(Pixy2SPI_SS &pixy, pixyLineVector (&vectorData)[2]){
//	int laneCenterX;
//	pixy.line.getAllFeatures(LINE_VECTOR, 1);
//	// if detects more than 2 vectors, calculate the center
//	if(pixy.line.numVectors >= 2)
//	    {
//	        // Determine left and right lines
//	        auto v1 = pixy.line.vectors[0];
//	        auto v2 = pixy.line.vectors[1];
//	        //writing vector data to struct for sd card logging
//	        vectorData[0].m_x0 = v1.m_x0;
//	        vectorData[0].m_y0 = v1.m_y0;
//	        vectorData[0].m_x1 = v1.m_x1;
//	        vectorData[0].m_y1 = v1.m_y1;
//
//	        vectorData[1].m_x0 = v2.m_x0;
//	        vectorData[1].m_y0 = v2.m_y0;
//	        vectorData[1].m_x1 = v2.m_x1;
//	        vectorData[1].m_y1 = v2.m_y1;
//	        //check if the 2 vectors valid
//	        //if valid, calculate using 2 vector logic
//		    if (twoVectorsValid(v1,v2)){
//		    	//2 vector logic
//		    	//calculate mid x coordinates
//		    	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
//		    	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
//		    	//compare midX1 and midX2, the smaller one is leftX and the bigger one is rightX
//		    	int leftX = (midX1 < midX2) ? midX1 : midX2;
//		    	int rightX = (midX1 < midX2) ? midX2 : midX1;
//
//		    	laneCenterX = (leftX + rightX)/2;
//		    	lastLaneCenterX = laneCenterX;
//		    }
//		    //if not valid, fallback to 1 vector logic
//		    else{
//		    	// use single vector logic
//		    	int laneCenterEstimate = singleVectorLogic(v1);
//		    	// smooth the vector
//		    	lastLaneCenterX = singleVectorSmooth(v1,laneCenterEstimate);
//		    	laneCenterX = lastLaneCenterX;
//		    }
//	    }
//	// only detects 1 vector
//	else if (pixy.line.numVectors == 1)
//	{
//	    auto v = pixy.line.vectors[0];
//	    // use single vector logic
//	    int laneCenterEstimate = singleVectorLogic(v);
//	    // smooth the vector
//	    lastLaneCenterX = singleVectorSmooth(v,laneCenterEstimate);
//	    laneCenterX = lastLaneCenterX;
//	}
//	//detects no vector
//	else{
//		// use last lane center
//		laneCenterX = 39;;
//	}
//
//	// error calculation
//	float error = (float)(laneCenterX - frameCenterX);
//    // Moving Average
//	//add error to the buffer array
//    errorBuffer[bufferIndex] = error;
//    //bufferIndex point to next element, use modulo to loop to element0 if window size is reached
//    bufferIndex = (bufferIndex + 1) % MA_WINDOW_SIZE;
//
//    if(bufferCount < MA_WINDOW_SIZE)
//        {bufferCount++;}
//
//    float sum = 0;
//    for(int i=0; i<bufferCount; i++)
//        {sum += errorBuffer[i];}
//
//    float avgError = sum / bufferCount;
//
//    //PD Controller, kP and kD defined as constant
//    float dError = avgError - lastAvgError;
//    lastAvgError = avgError;
//
//    float steer = kP * avgError + kD * dError;
//
//    // Limit the maximum range of steer, steerMax defined as constant
//    // to do: use clampf instead
//    if(steer > steerMax) steer = steerMax;
//    if(steer < -steerMax) steer = -steerMax;
//
//    // Limit turn rate. steerStepLimit defined as constant
//    float delta = steer - lastSteer;
//    if(delta > steerStepLimit)
//        {steer = lastSteer + steerStepLimit;}
//    else if(delta < -steerStepLimit)
//        {steer = lastSteer - steerStepLimit;}
//
//    lastSteer = steer;
//
//    return steer;
//}
