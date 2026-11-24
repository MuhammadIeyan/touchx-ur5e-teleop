// Offline regression test. All device operations reached by sample() are mocked.
#define main probe_program_main
#include "../../scripts/touch_probe.cpp"
#undef main
#include <cassert>

static HDErrorInfo pending{};
static HDerror updateError;
static int internalError;
static bool updated, forceEnabled, verifyError;
static HDenum afterUpdate;
extern "C" {
HDenum hdCheckCalibration() {
    if (!updated) return HD_CALIBRATION_NEEDS_UPDATE;
    if (verifyError) pending.errorCode = HD_COMM_ERROR;
    return afterUpdate;
}
void hdUpdateCalibration(HDenum) {
    updated = true;
    pending.errorCode = updateError;
    pending.internalErrorCode = internalError;
}
HDErrorInfo hdGetError() { auto value = pending; pending = {}; return value; }
HDboolean hdIsEnabled(HDenum) { return forceEnabled; }
void hdDisable(HDenum) { forceEnabled=false; }
void hdBeginFrame(HHD) {}
void hdEndFrame(HHD) {}
void hdGetDoublev(HDenum field, HDdouble* values) {
    for (int i = 0; i < (field == HD_CURRENT_TRANSFORM ? 16 : 3); ++i) values[i] = 0;
}
void hdGetIntegerv(HDenum, HDint* value) { *value = 0; }
}
static void check(bool optIn, HDerror code, int internal, bool forces,
                  HDenum status, bool verificationFails, bool expectedAccepted) {
    pending = {}; updated = false; updateError = code; internalError = internal;
    forceEnabled = forces; afterUpdate = status; verifyError = verificationFails;
    State state{};
    state.calibrationStyle = HD_CALIBRATION_AUTO;
    state.calibrationCompat = optIn;
    auto result = sample(&state);
    assert((result == HD_CALLBACK_CONTINUE) == expectedAccepted);
    assert(HD_DEVICE_ERROR(state.latest.error) == !expectedAccepted);
}
int main() {
    check(true, HD_INVALID_OPERATION, 1, false, HD_CALIBRATION_OK, false, true);
    check(false, HD_INVALID_OPERATION, 1, false, HD_CALIBRATION_OK, false, false);
    check(true, HD_INVALID_OPERATION, 2, false, HD_CALIBRATION_OK, false, false);
    check(true, HD_COMM_ERROR, 1, false, HD_CALIBRATION_OK, false, false);
    check(true, HD_INVALID_OPERATION, 1, true, HD_CALIBRATION_OK, false, true);
    check(true, HD_INVALID_OPERATION, 1, false, HD_CALIBRATION_NEEDS_MANUAL_INPUT, false, false);
    check(true, HD_INVALID_OPERATION, 1, false, HD_CALIBRATION_OK, true, false);
    check(false, HD_SUCCESS, 0, false, HD_CALIBRATION_OK, false, true);
    std::cout << "Calibration compatibility checks passed (8 cases).\n";
}
