#ifndef EYESTATE_H
#define EYESTATE_H

#include "consts.h"
#include <vector>

class QPointF;
class QColor;


enum MoveFlags {
    EyePosition           =  0x001,
    EyeRotation           =  0x002,
    EyeSize               =  0x004,
    EyeColors             =  0x008,
    PupilSize             =  0x010,
    PupilRotation         =  0x020,
    TopEyelidHeight       =  0x040,
    TopEyelidRotation     =  0x080,
    BottomEyelidHeight    =  0x100,
    BottomEyelidRotation  =  0x200,
};

enum EyeSide {
    UNSET,
    LEFT,
    RIGHT
};

struct EyeState {
    QPointF center = QPointF(WIDTH/2.,HEIGHT/2.);

    EyeSide side = UNSET;

    // Eye configuraton
    float radius       = 287.5;       // Eye's first radius
    float radius2      = radius * radiusRatio;  // Eye's second radius (it is elliptic)
    float pupilRadius  = 0.6;         // Pupil contraction radius
    float pupilAngle   = 0;           // Pupil rotation
    float angle        = 40.83*0.9;   // Eye rotation angle
    float radiusRatio  = 0.8;         // Radius1/Radius2

    // Eyelids
    float topEyelidAngle       = -5;
    float topEyelidY           = 135;
    float bottomEyelidAngle    = -5;
    float bottomEyelidY        = 740;

    // Colors
    QColor eyeColor            = Qt::green;
    QColor eyelidColor         = QColor(143, 210, 143);
    QColor eyelidOutlineColor  = QColor(116, 169, 116);
    QColor whiteAreaColor      = Qt::white;

    EyeState(bool isLeftEye = false) {
        if (isLeftEye)  side = LEFT;
        else            side = RIGHT; 

        resetConfiguration();
    }

    inline void resetConfiguration() {
        radiusRatio       = 0.8;
        radius            = 287.5;
        radius2           = radius * radiusRatio;
        pupilRadius       = 0.6;
        pupilAngle        = 0;
        angle             = 40.83*0.9;
        if (side == RIGHT)  angle = -angle;

        topEyelidAngle    = -5;
        topEyelidY        = 135;
        bottomEyelidAngle = -5;
        bottomEyelidY     = 740;
    }

    inline void resetColors() {
        eyeColor           = Qt::green;
        eyelidColor        = QColor(143, 210, 143);
        eyelidOutlineColor = QColor(116, 169, 116);
        whiteAreaColor     = Qt::white;
    }

    inline void assignOnlyBlinkState(EyeState source) {
        pupilRadius       = source.pupilRadius;
        topEyelidAngle    = source.topEyelidAngle;
        topEyelidY        = source.topEyelidY;
        bottomEyelidAngle = source.bottomEyelidAngle;
        bottomEyelidY     = source.bottomEyelidY;
    }

    // @Note: All color assigned at once, so they all must be valid!
    //        Maybe should rethink the process later.
    inline void assignSelectively(EyeState source, MoveFlags f) {
        if (f & EyePosition)  center = source.center;
        if (f & EyeRotation)  angle  = source.angle;
        if (f & EyeSize) {
            radius      = source.radius;
            radiusRatio = source.radiusRatio;
        }
        if (f & PupilSize)             pupilRadius       = source.pupilRadius;
        if (f & PupilRotation)         pupilAngle        = source.pupilAngle;
        if (f & TopEyelidHeight)       topEyelidY        = source.topEyelidY;
        if (f & TopEyelidRotation)     topEyelidAngle    = source.topEyelidAngle;
        if (f & BottomEyelidHeight)    bottomEyelidY     = source.bottomEyelidY;
        if (f & BottomEyelidRotation)  bottomEyelidAngle = source.bottomEyelidAngle;

        if (f & EyeColors) {
            eyeColor            = source.eyeColor;
            eyelidColor         = source.eyelidColor;
            eyelidOutlineColor  = source.eyelidOutlineColor;
            whiteAreaColor      = source.whiteAreaColor;
        }
    }
};

struct EyeAnimation {
    EyeState initState;

    std::vector<EyeState> states;
    std::vector<int> msDurations;

    void setInitialState(EyeState state) {
        initState = state;
    }

    void appendState(EyeState state, int ms) {
        states.push_back(state);
        msDurations.push_back(ms);
    }

    void clear() {
        states.clear();
        msDurations.clear();
    }
};

struct EyesAnimation {
    EyeAnimation leftEyeAnimation;
    EyeAnimation rightEyeAnimation;

    void setInitialStates(EyeState leftState, EyeState rightState) {
        leftEyeAnimation.setInitialState(leftState);
        rightEyeAnimation.setInitialState(rightState);
    }

    void appendStates(EyeState leftState, EyeState rightState, int ms) {
        leftEyeAnimation.appendState(leftState, ms);
        rightEyeAnimation.appendState(rightState, ms);
    }
};


inline float lerp(float first_value, float second_value, float t) {
    return first_value * (1 - t) + second_value * t;
}

inline QPointF lerp(QPointF p0, QPointF p1, float t) {
    return QPointF(p0.x() * (1 - t) + p1.x() * t,
                   p0.y() * (1 - t) + p1.y() * t);
}

// Do linear interpolation of only spacial configuration of EyeState.
// Colors assigned to result from p0 only if t == 0, otherwise they assigned from p1 
inline EyeState lerp(EyeState s0, EyeState s1, float t) {
    EyeState result;

    result.center      = lerp(s0.center, s1.center, t);
    result.radiusRatio = lerp(s0.radiusRatio, s1.radiusRatio, t);
    result.radius      = lerp(s0.radius, s1.radius, t);
    result.pupilRadius = lerp(s0.pupilRadius, s1.pupilRadius, t);
    result.pupilAngle  = lerp(s0.pupilAngle, s1.pupilAngle, t);
    result.angle       = lerp(s0.angle, s1.angle, t);

    result.topEyelidAngle     = lerp(s0.topEyelidAngle, s1.topEyelidAngle, t);
    result.topEyelidY         = lerp(s0.topEyelidY, s1.topEyelidY, t);
    result.bottomEyelidAngle  = lerp(s0.bottomEyelidAngle, s1.bottomEyelidAngle, t);
    result.bottomEyelidY      = lerp(s0.bottomEyelidY, s1.bottomEyelidY, t);

    if (t == 0.0) {
        result.eyeColor           = s0.eyeColor;
        result.eyelidColor        = s0.eyelidColor;
        result.eyelidOutlineColor = s0.eyelidOutlineColor;
        result.whiteAreaColor     = s0.whiteAreaColor;
    } else {
        result.eyeColor           = s1.eyeColor;
        result.eyelidColor        = s1.eyelidColor;
        result.eyelidOutlineColor = s1.eyelidOutlineColor;
        result.whiteAreaColor     = s1.whiteAreaColor;
    }

    return result;
}

inline float bezier_1d_cubic(float u0, float u1, float t) {
    float square = t * t;
    float cube = square * t;

    float inv = 1 - t;
    float inv_square = inv * (1 - t);

    return 3*inv_square*t*u0 + 3*inv*square*u1 + cube;
}

#endif // EYESTATE_H
