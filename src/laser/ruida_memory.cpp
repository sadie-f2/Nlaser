// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The address names are ported from meerk40t/ruida/emulator.py
// (RuidaEmulator.mem_lookup, commit d511944); the values that function
// returns are the emulator's stubs and are deliberately not copied. See
// third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_memory.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace ncad::laser::ruida {

namespace {

// One name, or a range sharing a name. `suffix` null: a plain name (a range
// appends " +k", k the offset from first). `suffix` set: a numbered name,
// name + (addr - base) + suffix. Ascending by `first`, which is how
// MeerK40t's function lists them. Five addresses appear twice in its function
// (0x77, 0x117, 0x177, 0x178, 0x190); as in Python the first wins here, the
// second can never be reached there either.
struct NameRow {
    std::uint16_t first, last;
    const char* name;
    const char* suffix;
    int base;
};

constexpr NameRow kNames[] = {
    {0x0002, 0x0002, "Laser Info", nullptr, 0},
    {0x0003, 0x0003, "Machine Def", nullptr, 0},
    {0x0004, 0x0004, "IOEnable", nullptr, 0},
    {0x0005, 0x0005, "G0 Velocity", nullptr, 0},
    {0x000B, 0x000B, "Eng Facula", nullptr, 0},
    {0x000C, 0x000C, "Home Velocity", nullptr, 0},
    {0x000E, 0x000E, "Eng Vert Velocity", nullptr, 0},
    {0x0010, 0x0010, "System Control Mode", nullptr, 0},
    {0x0011, 0x0011, "Laser PWM Frequency 1", nullptr, 0},
    {0x0012, 0x0012, "Laser Min Power 1", nullptr, 0},
    {0x0013, 0x0013, "Laser Max Power 1", nullptr, 0},
    {0x0016, 0x0016, "Laser Attenuation", nullptr, 0},
    {0x0017, 0x0017, "Laser PWM Frequency 2", nullptr, 0},
    {0x0018, 0x0018, "Laser Min Power 2", nullptr, 0},
    {0x0019, 0x0019, "Laser Max Power 2", nullptr, 0},
    {0x001A, 0x001A, "Laser Standby Frequency 1", nullptr, 0},
    {0x001B, 0x001B, "Laser Standby Pulse 1", nullptr, 0},
    {0x001C, 0x001C, "Laser Standby Frequency 2", nullptr, 0},
    {0x001D, 0x001D, "Laser Standby Pulse 2", nullptr, 0},
    {0x001E, 0x001E, "Auto Type Space", nullptr, 0},
    {0x001F, 0x001F, "TriColor", nullptr, 0},
    {0x0020, 0x0020, "Axis Control Para 1", nullptr, 0},
    {0x0021, 0x0021, "Axis Precision 1", nullptr, 0},
    {0x0023, 0x0023, "Axis Max Velocity 1", nullptr, 0},
    {0x0024, 0x0024, "Axis Start Velocity 1", nullptr, 0},
    {0x0025, 0x0025, "Axis Max Acc 1", nullptr, 0},
    {0x0026, 0x0026, "Axis Range 1, Get Frame X", nullptr, 0},
    {0x0027, 0x0027, "Axis Btn Start Velocity 1", nullptr, 0},
    {0x0028, 0x0028, "Axis Btn Acc 1", nullptr, 0},
    {0x0029, 0x0029, "Axis Estp Acc 1", nullptr, 0},
    {0x002A, 0x002A, "Axis Home Offset 1", nullptr, 0},
    {0x002B, 0x002B, "Axis Backlash 1", nullptr, 0},
    {0x0030, 0x0030, "Axis Control Para 2", nullptr, 0},
    {0x0031, 0x0031, "Axis Precision 2", nullptr, 0},
    {0x0033, 0x0033, "Axis Max Velocity 2", nullptr, 0},
    {0x0034, 0x0034, "Axis Start Velocity 2", nullptr, 0},
    {0x0035, 0x0035, "Axis Max Acc 2", nullptr, 0},
    {0x0036, 0x0036, "Axis Range 2, Get Frame Y", nullptr, 0},
    {0x0037, 0x0037, "Axis Btn Start Velocity 2", nullptr, 0},
    {0x0038, 0x0038, "Axis Btn Acc 2", nullptr, 0},
    {0x0039, 0x0039, "Axis Estp Acc 2", nullptr, 0},
    {0x003A, 0x003A, "Axis Home Offset 2", nullptr, 0},
    {0x003B, 0x003B, "Axis Backlash 2", nullptr, 0},
    {0x0040, 0x0040, "Axis Control Para 3", nullptr, 0},
    {0x0041, 0x0041, "Axis Precision 3", nullptr, 0},
    {0x0043, 0x0043, "Axis Max Velocity 3", nullptr, 0},
    {0x0044, 0x0044, "Axis Start Velocity 3", nullptr, 0},
    {0x0045, 0x0045, "Axis Max Acc 3", nullptr, 0},
    {0x0046, 0x0046, "Axis Range 3, Get Frame Z", nullptr, 0},
    {0x0047, 0x0047, "Axis Btn Start Velocity 3", nullptr, 0},
    {0x0048, 0x0048, "Axis Btn Acc 3", nullptr, 0},
    {0x0049, 0x0049, "Axis Estp Acc 3", nullptr, 0},
    {0x004A, 0x004A, "Axis Home Offset 3", nullptr, 0},
    {0x004B, 0x004B, "Axis Backlash 3", nullptr, 0},
    {0x0050, 0x0050, "Axis Control Para 4", nullptr, 0},
    {0x0051, 0x0051, "Axis Precision 4", nullptr, 0},
    {0x0053, 0x0053, "Axis Max Velocity 4", nullptr, 0},
    {0x0054, 0x0054, "Axis Start Velocity 4", nullptr, 0},
    {0x0055, 0x0055, "Axis Max Acc 4", nullptr, 0},
    {0x0056, 0x0056, "Axis Range 4, Get Frame U", nullptr, 0},
    {0x0057, 0x0057, "Axis Btn Start Velocity 4", nullptr, 0},
    {0x0058, 0x0058, "Axis Btn Acc 4", nullptr, 0},
    {0x0059, 0x0059, "Axis Estp Acc 4", nullptr, 0},
    {0x005A, 0x005A, "Axis Home Offset 4", nullptr, 0},
    {0x005B, 0x005B, "Axis Backlash 4", nullptr, 0},
    {0x0060, 0x0060, "Machine Type", nullptr, 0},
    {0x0063, 0x0063, "Laser Min Power 3", nullptr, 0},
    {0x0064, 0x0064, "Laser Max Power 3", nullptr, 0},
    {0x0065, 0x0065, "Laser PWM Frequency 3", nullptr, 0},
    {0x0066, 0x0066, "Laser Standby Frequency 3", nullptr, 0},
    {0x0067, 0x0067, "Laser Standby Pulse 3", nullptr, 0},
    {0x0068, 0x0068, "Laser Min Power 4", nullptr, 0},
    {0x0069, 0x0069, "Laser Max Power 4", nullptr, 0},
    {0x006A, 0x006A, "Laser PWM Frequency 4", nullptr, 0},
    {0x006B, 0x006B, "Laser Standby Frequency 4", nullptr, 0},
    {0x006C, 0x006C, "Laser Standby Pulse 4", nullptr, 0},
    {0x006D, 0x006D, "Laser Min Power 5", nullptr, 0},
    {0x006E, 0x006E, "Laser Max Power 5", nullptr, 0},
    {0x006F, 0x006F, "Laser PWM Frequency 5", nullptr, 0},
    {0x0070, 0x0070, "Laser Standby Frequency 5", nullptr, 0},
    {0x0071, 0x0071, "Laser Standby Pulse 5", nullptr, 0},
    {0x0072, 0x0072, "Laser Min Power 6", nullptr, 0},
    {0x0073, 0x0073, "Laser Max Power 6", nullptr, 0},
    {0x0074, 0x0074, "Laser PWM Frequency 6", nullptr, 0},
    {0x0075, 0x0075, "Laser Standby Frequency 6", nullptr, 0},
    {0x0076, 0x0076, "Laser Standby Pulse 6", nullptr, 0},
    {0x0077, 0x0077, "Auto Type Space 2", nullptr, 0},
    {0x0078, 0x0078, "Auto Type Space 4", nullptr, 0},
    {0x0079, 0x0079, "Auto Type Space 5", nullptr, 0},
    {0x007A, 0x007A, "Auto Type Space 6", nullptr, 0},
    {0x0080, 0x0080, "RD-UNKNOWN 2", nullptr, 0},
    {0x0090, 0x0090, "RD-UNKNOWN 3", nullptr, 0},
    {0x00A0, 0x00A0, "RD-UNKNOWN 4", nullptr, 0},
    {0x00B0, 0x00B0, "RD-UNKNOWN 5", nullptr, 0},
    {0x00C0, 0x00C0, "Offset 8 Start", nullptr, 0},
    {0x00C1, 0x00C1, "Offset 8 End", nullptr, 0},
    {0x00C2, 0x00C2, "Offset 9 Start", nullptr, 0},
    {0x00C3, 0x00C3, "Offset 9 End", nullptr, 0},
    {0x00C4, 0x00C4, "Offset 10 Start", nullptr, 0},
    {0x00C5, 0x00C5, "Offset 10 End", nullptr, 0},
    {0x00C6, 0x00C6, "Offset 7 Start", nullptr, 0},
    {0x00C7, 0x00C7, "Offset 7 End", nullptr, 0},
    {0x00C8, 0x00C8, "Axis Home Velocity 1", nullptr, 0},
    {0x00C9, 0x00C9, "Axis Home Velocity 2", nullptr, 0},
    {0x00CA, 0x00CA, "Margin 1", nullptr, 0},
    {0x00CB, 0x00CB, "Margin 2", nullptr, 0},
    {0x00CC, 0x00CC, "Margin 3", nullptr, 0},
    {0x00CD, 0x00CD, "Margin 4", nullptr, 0},
    {0x00CE, 0x00CE, "VWheelRatio", nullptr, 0},
    {0x00CF, 0x00CF, "VPunchRatio", nullptr, 0},
    {0x00D0, 0x00D0, "In Hale Zone", nullptr, 0},
    {0x00D8, 0x00D8, "VSlotRatio", nullptr, 0},
    {0x00D9, 0x00D9, "VSlot Share Home Offset", nullptr, 0},
    {0x00DA, 0x00DA, "VPunch Share Home Offset", nullptr, 0},
    {0x00E7, 0x00E7, "VWheel Share Home Offset", nullptr, 0},
    {0x0100, 0x0100, "System Settings", nullptr, 0},
    {0x0101, 0x0101, "Turn Velocity", nullptr, 0},
    {0x0102, 0x0102, "Syn Acc", nullptr, 0},
    {0x0103, 0x0103, "G0 Delay", nullptr, 0},
    {0x0104, 0x0104, "Scan Step Factor", nullptr, 0},
    {0x0105, 0x0105, "User Para 5", nullptr, 0},
    {0x0107, 0x0107, "Feed Delay After", nullptr, 0},
    {0x0108, 0x0108, "User Key Fast Velocity", nullptr, 0},
    {0x0109, 0x0109, "Turn Acc", nullptr, 0},
    {0x010A, 0x010A, "G0 Acc", nullptr, 0},
    {0x010B, 0x010B, "Feed Delay Prior", nullptr, 0},
    {0x010C, 0x010C, "Manual Distance", nullptr, 0},
    {0x010D, 0x010D, "Shut Down Delay", nullptr, 0},
    {0x010E, 0x010E, "Focus Depth", nullptr, 0},
    {0x010F, 0x010F, "Go Scale Blank", nullptr, 0},
    {0x0115, 0x0115, "Dock Point X", nullptr, 0},
    {0x0116, 0x0116, "Dock Point Y", nullptr, 0},
    {0x0117, 0x0117, "Array Feed Repay", nullptr, 0},
    {0x0119, 0x0119, "Rotate Off Delay", nullptr, 0},
    {0x011A, 0x011A, "Acc Ratio", nullptr, 0},
    {0x011B, 0x011B, "Turn Ratio", nullptr, 0},
    {0x011C, 0x011C, "Acc G0 Ratio", nullptr, 0},
    {0x011F, 0x011F, "Rotate Pulse", nullptr, 0},
    {0x0121, 0x0121, "Rotate D", nullptr, 0},
    {0x0122, 0x0122, "Eng Facula Replay", nullptr, 0},
    {0x0124, 0x0124, "X Min Eng Velocity", nullptr, 0},
    {0x0125, 0x0125, "X Eng Acc", nullptr, 0},
    {0x0126, 0x0126, "User Para 1", nullptr, 0},
    {0x0128, 0x0128, "Z Home Velocity", nullptr, 0},
    {0x0129, 0x0129, "Z Work Velocity", nullptr, 0},
    {0x012A, 0x012A, "Z G0 Velocity", nullptr, 0},
    {0x012B, 0x012B, "Union Home Distance", nullptr, 0},
    {0x012C, 0x012C, "U Home Velocity", nullptr, 0},
    {0x012D, 0x012D, "U Work Velocity", nullptr, 0},
    {0x012E, 0x012E, "Feed Repay", nullptr, 0},
    {0x0131, 0x0131, "Manual Fast Speed", nullptr, 0},
    {0x0132, 0x0132, "Manual Slow Speed", nullptr, 0},
    {0x0134, 0x0134, "Y Minimum Eng Velocity", nullptr, 0},
    {0x0135, 0x0135, "Y Eng Acc", nullptr, 0},
    {0x0137, 0x0137, "Eng Acc Ratio", nullptr, 0},
    {0x0138, 0x0138, "Sts Ahead Time", nullptr, 0},
    {0x0139, 0x0139, "Repeat Delay", nullptr, 0},
    {0x013B, 0x013B, "User Para 3", nullptr, 0},
    {0x013D, 0x013D, "User Para 2", nullptr, 0},
    {0x013F, 0x013F, "User Para 4", nullptr, 0},
    {0x0140, 0x0140, "Axis Home Velocity 3", nullptr, 0},
    {0x0141, 0x0141, "Axis Work Velocity 3", nullptr, 0},
    {0x0142, 0x0142, "Axis Home Velocity 4", nullptr, 0},
    {0x0143, 0x0143, "Axis Work Velocity 4", nullptr, 0},
    {0x0144, 0x0144, "Axis Home Velocity 5", nullptr, 0},
    {0x0145, 0x0145, "Axis Work Velocity 5", nullptr, 0},
    {0x0146, 0x0146, "Axis Home Velocity 6", nullptr, 0},
    {0x0147, 0x0147, "Axis Work Velocity 6", nullptr, 0},
    {0x0148, 0x0148, "Axis Home Velocity 7", nullptr, 0},
    {0x0149, 0x0149, "Axis Work Velocity 7", nullptr, 0},
    {0x014A, 0x014A, "Axis Home Velocity 8", nullptr, 0},
    {0x014B, 0x014B, "Axis Work Velocity 8", nullptr, 0},
    {0x014C, 0x014C, "Laser Reset Time", nullptr, 0},
    {0x014D, 0x014D, "Laser Start Distance", nullptr, 0},
    {0x014E, 0x014E, "Z Pen Up Pos", nullptr, 0},
    {0x014F, 0x014F, "Z Pen Down Pos", nullptr, 0},
    {0x0150, 0x0150, "Offset 1 Start", nullptr, 0},
    {0x0151, 0x0151, "Offset 1 End", nullptr, 0},
    {0x0152, 0x0152, "Offset 2 Start", nullptr, 0},
    {0x0153, 0x0153, "Offset 2 End", nullptr, 0},
    {0x0154, 0x0154, "Offset 3 Start", nullptr, 0},
    {0x0155, 0x0155, "Offset 3 End", nullptr, 0},
    {0x0156, 0x0156, "Offset 6 Start", nullptr, 0},
    {0x0157, 0x0157, "Offset 6 End", nullptr, 0},
    {0x0158, 0x0158, "Offset 4 Start", nullptr, 0},
    {0x0159, 0x0159, "Offset 4 End", nullptr, 0},
    {0x015A, 0x015A, "Offset 5 Start", nullptr, 0},
    {0x015B, 0x015B, "Offset 5 End", nullptr, 0},
    {0x015C, 0x015C, "Delay 6 On", nullptr, 0},
    {0x015D, 0x015D, "Delay 6 Off", nullptr, 0},
    {0x015E, 0x015E, "Delay 7 On", nullptr, 0},
    {0x015F, 0x015F, "Delay 7 Off", nullptr, 0},
    {0x0160, 0x0160, "Inhale On Delay", nullptr, 0},
    {0x0161, 0x0161, "Inhale Off Delay", nullptr, 0},
    {0x0162, 0x0162, "Delay 5 On", nullptr, 0},
    {0x0163, 0x0163, "Delay 5 Off", nullptr, 0},
    {0x0164, 0x0164, "Delay 2 On", nullptr, 0},
    {0x0165, 0x0165, "Delay 2 Off", nullptr, 0},
    {0x0166, 0x0166, "VSample Distance", nullptr, 0},
    {0x0169, 0x0169, "Offset 11 Start", nullptr, 0},
    {0x016A, 0x016A, "Offset 11 End", nullptr, 0},
    {0x016B, 0x016B, "Tool Up Pos 4", nullptr, 0},
    {0x016C, 0x016C, "Tool Down Pos 4", nullptr, 0},
    {0x016D, 0x016D, "VUp Angle", nullptr, 0},
    {0x016E, 0x016E, "VRotatePulse", nullptr, 0},
    {0x0170, 0x0170, "Delay 1 On", nullptr, 0},
    {0x0171, 0x0171, "VCorner Precision", nullptr, 0},
    {0x0172, 0x0172, "Delay 3 On", nullptr, 0},
    {0x0173, 0x0173, "Delay 4 Off", nullptr, 0},
    {0x0174, 0x0174, "Delay 4 On", nullptr, 0},
    {0x0175, 0x0175, "Delay 1 Off", nullptr, 0},
    {0x0176, 0x0176, "Delay 3 Off", nullptr, 0},
    {0x0177, 0x0177, "Delay 9 On", nullptr, 0},
    {0x0178, 0x0178, "Delay 9 Off", nullptr, 0},
    {0x0179, 0x0179, "Tool Down Pos 3", nullptr, 0},
    {0x017A, 0x017A, "Tool Up Pos 2", nullptr, 0},
    {0x017B, 0x017B, "Tool Down Pos 2", nullptr, 0},
    {0x017C, 0x017C, "Punch Rotate Delay", nullptr, 0},
    {0x017D, 0x017D, "VSlot Angle", nullptr, 0},
    {0x017E, 0x017E, "VTool Rotate Limit", nullptr, 0},
    {0x017F, 0x017F, "Tool Up Delay", nullptr, 0},
    {0x0180, 0x0180, "Card Language", nullptr, 0},
    {0x0181, 0x0187, "PC Lock ", "", 0x181},
    {0x0188, 0x0188, "User Key Slow Velocity", nullptr, 0},
    {0x0189, 0x0189, "MachineID 2", nullptr, 0},
    {0x018A, 0x018A, "MachineID 3", nullptr, 0},
    {0x018B, 0x018B, "MachineID 4", nullptr, 0},
    {0x018C, 0x018C, "Blow On Delay", nullptr, 0},
    {0x018D, 0x018D, "Blow Off Delay", nullptr, 0},
    {0x018F, 0x018F, "User Para 6, Blower", nullptr, 0},
    {0x0190, 0x0190, "Jet Time", nullptr, 0},
    {0x0191, 0x0191, "Color Mark Head Distance", nullptr, 0},
    {0x0192, 0x0192, "Color Mark Mark Distance", nullptr, 0},
    {0x0193, 0x0193, "Color Mark Camera Distance", nullptr, 0},
    {0x0194, 0x0194, "Color Mark Sensor Offset2", nullptr, 0},
    {0x0195, 0x0195, "Cylinder Down Delay", nullptr, 0},
    {0x0196, 0x0196, "Cylinder Up Delay", nullptr, 0},
    {0x0197, 0x0197, "Press Down Delay", nullptr, 0},
    {0x0198, 0x0198, "Press Up Delay", nullptr, 0},
    {0x0199, 0x0199, "Drop Position Start", nullptr, 0},
    {0x019A, 0x019A, "Drop Position End", nullptr, 0},
    {0x019B, 0x019B, "Drop Interval", nullptr, 0},
    {0x019C, 0x019C, "Drop Time", nullptr, 0},
    {0x019D, 0x019D, "Sharpen Delay On", nullptr, 0},
    {0x019E, 0x019E, "Sharpen Delay Off", nullptr, 0},
    {0x019F, 0x019F, "Sharpen Time Limit", nullptr, 0},
    {0x01A0, 0x01A0, "Sharpen Travel Limit", nullptr, 0},
    {0x01A1, 0x01A1, "Work End Time", nullptr, 0},
    {0x01A2, 0x01A2, "Color Mark Offset", nullptr, 0},
    {0x01A3, 0x01A3, "Color Mark Count", nullptr, 0},
    {0x01A4, 0x01A4, "Wheel Press Compensation", nullptr, 0},
    {0x01A5, 0x01A5, "Color Mark Filter Length", nullptr, 0},
    {0x01A6, 0x01A6, "VBlow Back On Delay", nullptr, 0},
    {0x01A7, 0x01A7, "VBlow Back Off Delay", nullptr, 0},
    {0x01AC, 0x01AC, "Y U Safe Distance", nullptr, 0},
    {0x01AD, 0x01AD, "Y U Home Distance", nullptr, 0},
    {0x01AE, 0x01AE, "VTool Preset Position X", nullptr, 0},
    {0x01AF, 0x01AF, "VTool Preset Position Y", nullptr, 0},
    {0x01B1, 0x01B1, "VTool Preset Compensation", nullptr, 0},
    {0x01B2, 0x01B2, "VTool Preset Cur Depth", nullptr, 0},
    {0x0200, 0x0200, "Machine Status", nullptr, 0},
    {0x0201, 0x0201, "Total Open Time (s)", nullptr, 0},
    {0x0202, 0x0202, "Total Work Time (s)", nullptr, 0},
    {0x0203, 0x0203, "Total Work Number", nullptr, 0},
    {0x0205, 0x0205, "Total Doc Number", nullptr, 0},
    {0x0206, 0x0206, "Flash Space", nullptr, 0},
    {0x0207, 0x0207, "Flash Space", nullptr, 0},
    {0x0208, 0x0208, "Previous Work Time", nullptr, 0},
    {0x0211, 0x0211, "Total Laser Work Time", nullptr, 0},
    {0x0212, 0x0212, "File Custom Flag / Feed Info", nullptr, 0},
    {0x0217, 0x0217, "Total Laser Work Time 2", nullptr, 0},
    {0x0218, 0x0218, "Total Laser Work Time 3", nullptr, 0},
    {0x0219, 0x0219, "Total Laser Work Time 4", nullptr, 0},
    {0x021A, 0x021A, "Total Laser Work Time 5", nullptr, 0},
    {0x021F, 0x021F, "Ring Number", nullptr, 0},
    {0x0221, 0x0221, "Axis Preferred Position 1, Pos X", nullptr, 0},
    {0x0223, 0x0223, "X Total Travel (m)", nullptr, 0},
    {0x0224, 0x0224, "Position Point 0", nullptr, 0},
    {0x0231, 0x0231, "Axis Preferred Position 2, Pos Y", nullptr, 0},
    {0x0233, 0x0233, "Y Total Travel (m)", nullptr, 0},
    {0x0234, 0x0234, "Position Point 1", nullptr, 0},
    {0x0241, 0x0241, "Axis Preferred Position 3, Pos Z", nullptr, 0},
    {0x0243, 0x0243, "Z Total Travel (m)", nullptr, 0},
    {0x0251, 0x0251, "Axis Preferred Position 4, Pos U", nullptr, 0},
    {0x0253, 0x0253, "U Total Travel (m)", nullptr, 0},
    {0x025A, 0x025A, "Axis Preferred Position 5, Pos A", nullptr, 0},
    {0x025B, 0x025B, "Axis Preferred Position 6, Pos B", nullptr, 0},
    {0x025C, 0x025C, "Axis Preferred Position 7, Pos C", nullptr, 0},
    {0x025D, 0x025D, "Axis Preferred Position 8, Pos D", nullptr, 0},
    {0x0260, 0x0260, "DocumentWorkNum", nullptr, 0},
    {0x0261, 0x02C3, "Document Number", nullptr, 0},
    {0x02C4, 0x02C4, "Read Scan Backlash Flag", nullptr, 0},
    {0x02C5, 0x02C5, "Read Scan Backlash 1", nullptr, 0},
    {0x02D5, 0x02D5, "Read Scan Backlash 2", nullptr, 0},
    {0x02FE, 0x02FE, "Card ID", nullptr, 0},
    {0x02FF, 0x02FF, "Mainboard Version", nullptr, 0},
    {0x0313, 0x0313, "Material Thickness", nullptr, 0},
    {0x031C, 0x031C, "File Fault", nullptr, 0},
    {0x0320, 0x0320, "File Total Length", nullptr, 0},
    {0x0321, 0x0321, "File Progress Len", nullptr, 0},
    {0x033B, 0x033B, "Read Process Feed Length", nullptr, 0},
    {0x0340, 0x0340, "Stop Time", nullptr, 0},
    {0x0391, 0x041F, "Time for File ", " to Run", 0x390},
    {0x0591, 0x0591, "Card Lock", nullptr, 0},
    {0x05C0, 0x05C0, "Laser Life", nullptr, 0},
};

const NameRow* find_row(std::uint16_t mem) {
    for (const NameRow& r : kNames) {
        if (mem >= r.first && mem <= r.last) return &r;
    }
    return nullptr;
}

}  // namespace

std::string memory_name(std::uint16_t mem) {
    const NameRow* r = find_row(mem);
    if (!r) return {};
    std::string n = r->name;
    if (r->suffix) {
        n += std::to_string(static_cast<int>(mem) - r->base);
        n += r->suffix;
    } else if (r->last != r->first) {
        n += " +" + std::to_string(mem - r->first);
    }
    return n;
}

std::vector<std::uint16_t> named_addresses() {
    std::vector<std::uint16_t> out;
    for (const NameRow& r : kNames) {
        for (unsigned a = r.first; a <= r.last; ++a) out.push_back(static_cast<std::uint16_t>(a));
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

bool memory_name_interesting(const std::string& name) {
    std::string l;
    for (char c : name) l.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (l.find("light") != std::string::npos || l.find("pointer") != std::string::npos ||
        l.find("offset") != std::string::npos) {
        return true;
    }
    for (std::size_t p = l.find("red"); p != std::string::npos; p = l.find("red", p + 1)) {
        const bool before = p == 0 || !std::isalpha(static_cast<unsigned char>(l[p - 1]));
        const bool after = p + 3 >= l.size() || !std::isalpha(static_cast<unsigned char>(l[p + 3]));
        if (before && after) return true;
    }
    return false;
}

bool parse_memory_address(const std::string& text, std::uint16_t& out) {
    auto hex = [](const std::string& s, unsigned long& v) {
        if (s.empty() || s.size() > 4) return false;
        v = 0;
        for (char c : s) {
            if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
            const int d = std::isdigit(static_cast<unsigned char>(c)) ? c - '0' : std::tolower(c) - 'a' + 10;
            v = v * 16 + static_cast<unsigned long>(d);
        }
        return true;
    };
    unsigned long v = 0;
    const std::size_t colon = text.find(':');
    if (colon != std::string::npos) {
        unsigned long hi = 0, lo = 0;
        if (!hex(text.substr(0, colon), hi) || !hex(text.substr(colon + 1), lo) || hi > 0x7F || lo > 0x7F) return false;
        v = (hi << 7) | lo;
    } else {
        const bool prefixed = text.compare(0, 2, "0x") == 0 || text.compare(0, 2, "0X") == 0;
        if (!hex(prefixed ? text.substr(2) : text, v) || v > 0x3FFF) return false;
    }
    out = static_cast<std::uint16_t>(v);
    return true;
}

std::string format_memory_address(std::uint16_t mem) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "0x%04X [%02X %02X]", static_cast<unsigned>(mem),
                  static_cast<unsigned>((mem >> 7) & 0x7Fu), static_cast<unsigned>(mem & 0x7Fu));
    return buf;
}

std::string value_as_text(const std::string& raw) {
    if (raw.size() <= 5) return {};
    std::string s;
    for (char c : raw) {
        if (c == '\0') break;
        if (c < 0x20 || c >= 0x7F) return {};
        s.push_back(c);
    }
    return s.size() >= 3 ? s : std::string();
}

bool read_settings(RuidaTransport& t, const std::vector<std::uint16_t>& addresses, std::vector<SettingRow>& rows,
                   const std::function<bool()>& stop, const std::function<void(std::size_t, std::size_t)>& progress,
                   std::string& error) {
    (void)error;  // a missing reply is a row, not a failure
    rows.clear();
    for (std::size_t i = 0; i < addresses.size(); ++i) {
        if (stop && stop()) break;
        SettingRow row;
        row.addr = addresses[i];
        row.name = memory_name(row.addr);
        row.interesting = memory_name_interesting(row.name);
        MemReply r;
        std::string why;
        if (t.read_memory(row.addr, r, why)) {
            row.replied = true;
            row.raw = r.raw;
            row.has_number = r.has_number;
            row.number = r.number;
            row.text = value_as_text(r.raw);
        } else {
            row.error = why;
        }
        rows.push_back(std::move(row));
        if (progress) progress(i + 1, addresses.size());
    }
    return true;
}

}  // namespace ncad::laser::ruida
