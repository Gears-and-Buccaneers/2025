#include <vector>
#include <stdio.h>

#include "lidar_jni.h"
#include "sl_lidar.h"

#undef FINDLINE_DEBUG
#include "findline.h"

#define GET(name, expr) name = env->expr; if (name == nullptr) { \
    printf(#name ": " #expr " is NULL!");                        \
    return JNI_ERR;                                              \
}

#define WIDTH 0.94

#ifdef _WIN32
#define CHANNEL "\\\\.\\com3"
#elif __APPLE__
#define CHANNEL "/dev/tty.SLAB_USBtoUART"
#else
#define CHANNEL "/dev/ttyUSB0"
#endif

#define BAUDRATE 115200

#define JNI_VERSION JNI_VERSION_1_6

static jclass Lidar;
static jfieldID Lidar_statePtr;
static jmethodID Lidar_callback;

static jclass Rotation2d;
static jclass Transform2d;

static jmethodID Rotation2d_init;
static jmethodID Transform2d_init;

static sl_lidar_response_measurement_node_hq_t nodes[8192];

struct State {
    sl::IChannel* chan;
    sl::ILidarDriver* drv;

    sl_u16 scan_mode;
};


jint JNI_OnLoad(JavaVM *vm, __attribute__((unused)) void *reserved) {
    JNIEnv* env = 0;

    if (vm->GetEnv((void**)&env, JNI_VERSION) != JNI_OK) return JNI_ERR;

    // Load global class references.
    jclass ctmp;

    GET(ctmp, FindClass("frc/robot/subsystems/Lidar"));
    Lidar = (jclass) env->NewGlobalRef(ctmp);
    env->DeleteLocalRef(ctmp);

    GET(ctmp, FindClass("edu/wpi/first/math/geometry/Rotation2d"));
    Rotation2d = (jclass) env->NewGlobalRef(ctmp);
    env->DeleteLocalRef(ctmp);
    
    GET(ctmp, FindClass("edu/wpi/first/math/geometry/Transform2d"));
    Transform2d = (jclass) env->NewGlobalRef(ctmp);
    env->DeleteLocalRef(ctmp);

    // Load class fields.
    GET(Lidar_statePtr, GetFieldID(Lidar, "statePtr", "J"));

    // Load class methods.
    GET(Lidar_callback, GetMethodID(Lidar, "callback", "(Ledu/wpi/first/math/geometry/Transform2d;)V"));

    // Load class constructors.
    GET(Rotation2d_init, GetMethodID(Rotation2d, "<init>", "(DD)V"));
    GET(Transform2d_init, GetMethodID(Transform2d, "<init>", "(DDLedu/wpi/first/math/geometry/Rotation2d;)V"));

    return JNI_VERSION;
}

jlong Java_frc_robot_subsystems_Lidar_construct(__attribute__((unused)) JNIEnv* env, __attribute__((unused)) jobject self) {
    State* st = (State*) malloc(sizeof(State));

    auto chan = sl::createSerialPortChannel(CHANNEL, BAUDRATE);
    auto drv = sl::createLidarDriver();

    if (!chan || !drv) {
        printf("LIDAR: Could not instantiate serial port and driver!\n");
        return 0;
    }

    st->drv = *drv;
    st->chan = *chan;

    if (SL_IS_FAIL(st->drv->connect(st->chan))) {
        printf("LIDAR: Could not connect!\n");
        return 0;
    }

    // Check lidar health.
	sl_lidar_response_device_health_t healthinfo;
    if (SL_IS_FAIL(st->drv->getHealth(healthinfo)) || healthinfo.status == SL_LIDAR_STATUS_ERROR) {
        printf("LIDAR: Health status failure!\n");
        return 0;
    }

    // Get the fastest scan mode.
	std::vector<sl::LidarScanMode> modes = {};

	if (SL_IS_FAIL(st->drv->getAllSupportedScanModes(modes)) || modes.empty()) {
        printf("LIDAR: Could not enumerate scan modes!\n");
        return 0;
    }

    float best = modes[0].us_per_sample;

    for (size_t i = 1; i < modes.size(); ++i) {
        if (modes[i].us_per_sample < best) {
            st->scan_mode = modes[i].id;
            best = modes[i].us_per_sample;
        }
    }

    // Return the state pointer as an integer.
    return (long) st;
}

void Java_frc_robot_subsystems_Lidar_startMotor(JNIEnv* env, jobject self) {
    State* st = (State*) env->GetLongField(self, Lidar_statePtr);
    if (st == nullptr) return;
    st->drv->setMotorSpeed(DEFAULT_MOTOR_SPEED);
}

void Java_frc_robot_subsystems_Lidar_initialize(JNIEnv* env, jobject self) {
    State* st = (State*) env->GetLongField(self, Lidar_statePtr);
    if (st == nullptr) return;
    st->drv->startScanExpress(0, st->scan_mode);
}

void Java_frc_robot_subsystems_Lidar_execute(JNIEnv* env, jobject self) {
    State* st = (State*) env->GetLongField(self, Lidar_statePtr);
    if (st == nullptr) return;

    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    if (SL_IS_FAIL(st->drv->grabScanDataHq(nodes, count, 0))) return;
    
    for (size_t i = 0; i < count; i++)
        nodes[i].angle_z_q14 = (1 << 14) - nodes[i].angle_z_q14;

    st->drv->ascendScanData(nodes, count);

    LineRes res;
    if (find_line(WIDTH, nodes + count - 1, nodes, &res) == -1) return;

    jobject rot = env->NewObject(Rotation2d, Rotation2d_init, res.rx, res.ry);
    jobject xfm = env->NewObject(Transform2d, Transform2d_init, res.cx, res.cy, rot);

    env->CallVoidMethod(self, Lidar_callback, xfm);
}

void Java_frc_robot_subsystems_Lidar_end(JNIEnv* env, jobject self, __attribute__((unused)) jboolean interrupted) {
    State* st = (State*) env->GetLongField(self, Lidar_statePtr);
    if (st == nullptr) return;
    st->drv->stop();
}

void Java_frc_robot_subsystems_Lidar_stopMotor(JNIEnv* env, jobject self) {
    State* st = (State*) env->GetLongField(self, Lidar_statePtr);
    if (st == nullptr) return;
    st->drv->setMotorSpeed(0);
}
