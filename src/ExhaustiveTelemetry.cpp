#include "ExhaustiveTelemetry.h"
#include "json.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

ExhaustiveTelemetryRecorder::ExhaustiveTelemetryRecorder(size_t reserveTicks) {
    samples.reserve(reserveTicks);
}

void ExhaustiveTelemetryRecorder::Reset() {
    samples.clear();
    summary = ExhaustiveSummary{};
    prevSteerInput = 0.0f;
}

void ExhaustiveTelemetryRecorder::RecordTick(const Car& car, 
                                            const FlatSpatialGrid& grid, 
                                            int tick, 
                                            const std::vector<Vector2>& checkpoints, 
                                            const std::vector<Vector2>& denseLine,
                                            float trackWidth) {
    ExhaustiveSample s{};
    s.tick = tick;
    s.position = car.position;
    s.rotation = car.rotation;
    s.vLong = car.longitudinalVelocity;
    s.vLat = car.lateralVelocity;
    s.speed = std::sqrt(car.velocity.x * car.velocity.x + car.velocity.y * car.velocity.y);
    s.angularVelocity = car.angularVelocity;
    s.longAccel = car.longAccel;
    s.latAccel = car.latAccel;
    s.jerkLong = car.jerkLong;
    s.jerkLat = car.jerkLat;
    s.slipAngle = car.slipAngle;
    s.kammUtil = car.kammUtilization;

    s.surface = car.currentSurface;
    s.surfaceFriction = car.currentSurfaceFriction;
    s.tireSurfaceTemp = car.tireThermals.surfaceTemp;
    s.tireBulkTemp = car.tireThermals.bulkTemp;
    s.tireThermalGrip = car.tireThermals.thermalGripFactor;

    // Neural inputs (12) and activations (10)
    for (int i = 0; i < INPUT_NODES; ++i) s.inputs[i] = car.brain.last_input[i];
    for (int i = 0; i < HIDDEN_NODES; ++i) s.hiddenActivations[i] = car.brain.last_hidden[i];
    s.neuralSaturation = car.brain.ComputeHiddenSaturation();

    // Outputs
    s.outThrottle = car.brain.last_output[0];
    s.outSteer = car.brain.last_output[1];

    // Jacobians
    float J[OUTPUT_NODES][INPUT_NODES];
    const_cast<Car&>(car).brain.ComputeJacobian(J);
    s.jacobianSteerFront = J[1][2];
    s.jacobianSteerLat = J[1][1] - J[1][3];
    s.jacobianThrottleFront = J[0][2];

    // Track Navigation & Offset
    if (!denseLine.empty()) {
        float minDistSq = 1e9f;
        int nearestIdx = 0;
        for (int i = 0; i < static_cast<int>(denseLine.size()); ++i) {
            float dx = denseLine[i].x - car.position.x;
            float dy = denseLine[i].y - car.position.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < minDistSq) {
                minDistSq = d2;
                nearestIdx = i;
            }
        }
        int nextIdx = (nearestIdx + 1) % static_cast<int>(denseLine.size());
        Vector2 tangent = { denseLine[nextIdx].x - denseLine[nearestIdx].x, denseLine[nextIdx].y - denseLine[nearestIdx].y };
        float tLen = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        if (tLen > 1e-4f) {
            tangent.x /= tLen;
            tangent.y /= tLen;
            Vector2 toCar = { car.position.x - denseLine[nearestIdx].x, car.position.y - denseLine[nearestIdx].y };
            float cross = tangent.x * toCar.y - tangent.y * toCar.x; // +: right of line, -: left of line
            s.trackOffset = cross;
            s.trackWidthRatio = std::clamp(cross / (trackWidth * 0.5f), -1.5f, 1.5f);
        }
    }

    if (!checkpoints.empty()) {
        int cpIdx = car.totalCheckPointsCrossed % static_cast<int>(checkpoints.size());
        s.currentCP = cpIdx;
        Vector2 targetCP = checkpoints[cpIdx];
        float dx = targetCP.x - car.position.x;
        float dy = targetCP.y - car.position.y;
        float angleToCP = std::atan2(dy, dx) * (180.0f / 3.14159265f);
        float angleDiff = angleToCP - car.rotation;
        while (angleDiff > 180.0f) angleDiff -= 360.0f;
        while (angleDiff < -180.0f) angleDiff += 360.0f;
        s.targetCPAngle = angleDiff;
    }

    s.checkpointsCrossed = car.totalCheckPointsCrossed;
    s.timeSinceLastCP = car.timeSinceLastCheckpoint;
    s.stallTicks = car.stallTicks;
    s.fitness = car.fitness;

    // Evaluate Behavioral Incoherences
    uint32_t flags = INCOH_NONE;
    if (car.sensorDistances[1] < 25.0f && s.outSteer < -0.2f) {
        flags |= INCOH_STEER_INTO_LEFT_WALL;
        summary.countSteerIntoWall++;
    }
    if (car.sensorDistances[3] < 25.0f && s.outSteer > 0.2f) {
        flags |= INCOH_STEER_INTO_RIGHT_WALL;
        summary.countSteerIntoWall++;
    }
    if (car.sensorDistances[2] < 80.0f && s.vLong > 2.0f && s.outThrottle > 0.5f) {
        flags |= INCOH_FULL_THROTTLE_WALL;
        summary.countFullThrottleWall++;
    }
    if (car.sensorDistances[2] > 250.0f && car.sensorDistances[1] > 40.0f && car.sensorDistances[3] > 40.0f && s.outThrottle < -0.3f) {
        flags |= INCOH_BRAKE_ON_STRAIGHT;
        summary.countBrakeOnStraight++;
    }
    if (std::abs(s.outSteer - prevSteerInput) > 1.2f) {
        flags |= INCOH_BANGBANG_STEERING;
        summary.countBangBangSteering++;
    }
    if (s.kammUtil > 1.2f && std::abs(s.slipAngle) > 25.0f) {
        flags |= INCOH_GRIP_COLLAPSE;
        summary.countGripCollapse++;
    }
    if (s.vLong < 0.05f && s.outThrottle < 0.0f) {
        flags |= INCOH_REVERSE_THRUST;
        summary.countReverseThrust++;
    }
    if (s.jacobianSteerLat > 0.10f) {
        flags |= INCOH_JACOBIAN_INVERTED;
        summary.countJacobianInverted++;
    }
    if (s.stallTicks > 30) {
        flags |= INCOH_STALLING;
        summary.countStalling++;
    }

    s.incoherenceFlags = flags;
    prevSteerInput = s.outSteer;

    samples.push_back(s);
}

void ExhaustiveTelemetryRecorder::Finalize(bool crashed, const std::string& crashReason) {
    summary.totalTicks = static_cast<int>(samples.size());
    summary.crashed = crashed;
    summary.crashTick = crashed ? summary.totalTicks : 0;
    summary.crashReason = crashReason;

    if (samples.empty()) return;

    summary.checkpointsCrossed = samples.back().checkpointsCrossed;
    summary.finalFitness = samples.back().fitness;

    double sumSpeed = 0.0, sumKamm = 0.0, sumSlip = 0.0, sumJerk = 0.0;
    summary.maxSpeed = 0.0f;
    summary.maxKammUtil = 0.0f;
    summary.maxSlipAngle = 0.0f;

    for (const auto& s : samples) {
        sumSpeed += s.speed;
        sumKamm += s.kammUtil;
        sumSlip += std::abs(s.slipAngle);
        sumJerk += std::sqrt(s.jerkLong * s.jerkLong + s.jerkLat * s.jerkLat);

        if (s.speed > summary.maxSpeed) summary.maxSpeed = s.speed;
        if (s.kammUtil > summary.maxKammUtil) summary.maxKammUtil = s.kammUtil;
        if (std::abs(s.slipAngle) > summary.maxSlipAngle) summary.maxSlipAngle = std::abs(s.slipAngle);
    }

    int N = summary.totalTicks;
    summary.avgSpeed = static_cast<float>(sumSpeed / N);
    summary.avgKammUtil = static_cast<float>(sumKamm / N);
    summary.avgSlipAngle = static_cast<float>(sumSlip / N);
    summary.avgJerk = static_cast<float>(sumJerk / N);

    summary.steeringCoherencePct = std::clamp(100.0f * (1.0f - static_cast<float>(summary.countSteerIntoWall) / static_cast<float>(N)), 0.0f, 100.0f);
    summary.throttleCoherencePct = std::clamp(100.0f * (1.0f - static_cast<float>(summary.countFullThrottleWall + summary.countBrakeOnStraight) / static_cast<float>(N)), 0.0f, 100.0f);
    summary.gripStabilityPct = std::clamp(100.0f * (1.0f - static_cast<float>(summary.countGripCollapse) / static_cast<float>(N)), 0.0f, 100.0f);

    float totalIncoherences = static_cast<float>(summary.countSteerIntoWall * 3 + 
                                                 summary.countFullThrottleWall * 3 + 
                                                 summary.countBrakeOnStraight * 2 + 
                                                 summary.countBangBangSteering * 1 + 
                                                 summary.countGripCollapse * 2 + 
                                                 summary.countReverseThrust * 1);
    summary.globalCoherenceScore = std::clamp(100.0f * (1.0f - totalIncoherences / static_cast<float>(N * 3)), 0.0f, 100.0f);
}

bool ExhaustiveTelemetryRecorder::ExportCSV(const std::string& filepath) const {
    std::ofstream f(filepath);
    if (!f.is_open()) return false;

    // Header with all 40+ attributes
    f << "tick,pos_x,pos_y,rotation,speed,v_long,v_lat,angular_vel,long_accel,lat_accel,jerk_long,jerk_lat,slip_angle,kamm_util,"
      << "surface_type,surface_friction,tire_surf_temp,tire_bulk_temp,tire_thermal_grip,"
      << "in_s0,in_s1,in_s2,in_s3,in_s4,in_ttc,in_vlong,in_vlat,in_yaw,in_opp_f,in_opp_l,in_opp_r,"
      << "h0,h1,h2,h3,h4,h5,h6,h7,h8,h9,neural_sat,"
      << "out_throttle,out_steer,j_steer_front,j_steer_lat,j_throttle_front,"
      << "track_offset,track_width_ratio,target_cp_angle,current_cp,cp_crossed,time_since_cp,stall_ticks,fitness,incoh_flags\n";

    for (const auto& s : samples) {
        f << s.tick << ","
          << std::fixed << std::setprecision(2) << s.position.x << "," << s.position.y << ","
          << s.rotation << "," << s.speed << "," << s.vLong << "," << s.vLat << ","
          << s.angularVelocity << "," << s.longAccel << "," << s.latAccel << ","
          << s.jerkLong << "," << s.jerkLat << "," << s.slipAngle << "," << s.kammUtil << ","
          << static_cast<int>(s.surface) << "," << s.surfaceFriction << ","
          << s.tireSurfaceTemp << "," << s.tireBulkTemp << "," << s.tireThermalGrip << ",";

        // 12 inputs
        for (int i = 0; i < INPUT_NODES; ++i) f << std::setprecision(4) << s.inputs[i] << ",";
        // 10 hidden activations
        for (int i = 0; i < HIDDEN_NODES; ++i) f << std::setprecision(4) << s.hiddenActivations[i] << ",";
        f << s.neuralSaturation << ",";

        // outputs & Jacobians
        f << std::setprecision(4) << s.outThrottle << "," << s.outSteer << ","
          << s.jacobianSteerFront << "," << s.jacobianSteerLat << "," << s.jacobianThrottleFront << ",";

        // track & evaluation
        f << std::setprecision(2) << s.trackOffset << "," << s.trackWidthRatio << "," << s.targetCPAngle << ","
          << s.currentCP << "," << s.checkpointsCrossed << "," << s.timeSinceLastCP << ","
          << s.stallTicks << "," << s.fitness << "," << s.incoherenceFlags << "\n";
    }

    return true;
}

bool ExhaustiveTelemetryRecorder::ExportJSON(const std::string& filepath) const {
    std::ofstream f(filepath);
    if (!f.is_open()) return false;

    nlohmann::json j;
    j["summary"]["totalTicks"] = summary.totalTicks;
    j["summary"]["checkpointsCrossed"] = summary.checkpointsCrossed;
    j["summary"]["finalFitness"] = summary.finalFitness;
    j["summary"]["avgSpeed"] = summary.avgSpeed;
    j["summary"]["maxSpeed"] = summary.maxSpeed;
    j["summary"]["avgKammUtil"] = summary.avgKammUtil;
    j["summary"]["maxKammUtil"] = summary.maxKammUtil;
    j["summary"]["avgSlipAngle"] = summary.avgSlipAngle;
    j["summary"]["maxSlipAngle"] = summary.maxSlipAngle;
    j["summary"]["avgJerk"] = summary.avgJerk;
    j["summary"]["crashed"] = summary.crashed;
    j["summary"]["crashTick"] = summary.crashTick;
    j["summary"]["crashReason"] = summary.crashReason;

    j["coherence_scores"]["steeringCoherencePct"] = summary.steeringCoherencePct;
    j["coherence_scores"]["throttleCoherencePct"] = summary.throttleCoherencePct;
    j["coherence_scores"]["gripStabilityPct"] = summary.gripStabilityPct;
    j["coherence_scores"]["globalCoherenceScore"] = summary.globalCoherenceScore;

    j["anomalies_count"]["steerIntoWall"] = summary.countSteerIntoWall;
    j["anomalies_count"]["fullThrottleWall"] = summary.countFullThrottleWall;
    j["anomalies_count"]["brakeOnStraight"] = summary.countBrakeOnStraight;
    j["anomalies_count"]["bangBangSteering"] = summary.countBangBangSteering;
    j["anomalies_count"]["gripCollapse"] = summary.countGripCollapse;
    j["anomalies_count"]["reverseThrust"] = summary.countReverseThrust;
    j["anomalies_count"]["jacobianInverted"] = summary.countJacobianInverted;
    j["anomalies_count"]["stalling"] = summary.countStalling;

    f << j.dump(4);
    return true;
}

void ExhaustiveTelemetryRecorder::PrintSummaryTable(std::ostream& os) const {
    os << "\n=======================================================================\n";
    os << "       EXHAUSTIVE TELEMETRY & BEHAVIORAL COHERENCE AUDIT REPORT         \n";
    os << "=======================================================================\n";
    os << " Ticks Evaluated:     " << summary.totalTicks << "\n";
    os << " Status:              " << (summary.crashed ? ("CRASHED at tick " + std::to_string(summary.crashTick) + " (" + summary.crashReason + ")") : "SURVIVED (Clean)") << "\n";
    os << " Checkpoints Crossed: " << summary.checkpointsCrossed << "\n";
    os << " Final Fitness:       " << std::fixed << std::setprecision(1) << summary.finalFitness << "\n";
    os << "-----------------------------------------------------------------------\n";
    os << " KINEMATICS & DYNAMICS:\n";
    os << "   Speed (Avg / Max): " << std::setprecision(2) << summary.avgSpeed << " / " << summary.maxSpeed << " px/tick\n";
    os << "   Kamm Utilization:  " << std::setprecision(2) << summary.avgKammUtil << " (Max: " << summary.maxKammUtil << ")\n";
    os << "   Slip Angle:        " << std::setprecision(1) << summary.avgSlipAngle << " deg (Max: " << summary.maxSlipAngle << " deg)\n";
    os << "   Control Jerk:      " << std::setprecision(3) << summary.avgJerk << " px/tick^3\n";
    os << "-----------------------------------------------------------------------\n";
    os << " BEHAVIORAL COHERENCE INDEXES (Target: > 95.0%):\n";
    os << "   Steering Coherence: " << std::setprecision(1) << summary.steeringCoherencePct << " %\n";
    os << "   Throttle Coherence: " << std::setprecision(1) << summary.throttleCoherencePct << " %\n";
    os << "   Tire Grip Stability:" << std::setprecision(1) << summary.gripStabilityPct << " %\n";
    os << "   GLOBAL COHERENCE:   " << std::setprecision(1) << summary.globalCoherenceScore << " %\n";
    os << "-----------------------------------------------------------------------\n";
    os << " INCOHERENCE ANOMALIES DETECTED:\n";
    os << "   Steer Into Wall:          " << summary.countSteerIntoWall << " ticks\n";
    os << "   Full Throttle Into Wall:  " << summary.countFullThrottleWall << " ticks\n";
    os << "   Braking on Open Straight: " << summary.countBrakeOnStraight << " ticks\n";
    os << "   Bang-Bang Steering Jerk:  " << summary.countBangBangSteering << " ticks\n";
    os << "   Tire Grip Collapses:      " << summary.countGripCollapse << " ticks\n";
    os << "   Reverse Thrust from Rest: " << summary.countReverseThrust << " ticks\n";
    os << "   Inverted Lateral Jacobian:" << summary.countJacobianInverted << " ticks\n";
    os << "   Stalling / Stagnation:    " << summary.countStalling << " ticks\n";
    os << "=======================================================================\n";
}

void ExhaustiveTelemetryRecorder::PrintAnomalyTimeline(std::ostream& os, int maxEntries) const {
    os << "\n--- ANOMALIES & COGNITIVE INCOHERENCE TIMELINE ---\n";
    int printed = 0;
    for (const auto& s : samples) {
        if (s.incoherenceFlags == INCOH_NONE) continue;

        os << "Tick " << std::setw(4) << s.tick 
           << " | Pos (" << std::fixed << std::setprecision(1) << s.position.x << ", " << s.position.y << ")"
           << " | Spd " << std::setprecision(2) << s.vLong
           << " | Acc " << s.outThrottle << " | Turn " << s.outSteer
           << " | S[L,C,R]: (" << (int)(s.inputs[1]*500) << ", " << (int)(s.inputs[2]*450) << ", " << (int)(s.inputs[3]*500) << ")"
           << " | Flags: ";

        if (s.incoherenceFlags & INCOH_STEER_INTO_LEFT_WALL) os << "[STEER->L_WALL] ";
        if (s.incoherenceFlags & INCOH_STEER_INTO_RIGHT_WALL) os << "[STEER->R_WALL] ";
        if (s.incoherenceFlags & INCOH_FULL_THROTTLE_WALL) os << "[FULL_THROTTLE_WALL] ";
        if (s.incoherenceFlags & INCOH_BRAKE_ON_STRAIGHT) os << "[BRAKE_STRAIGHT] ";
        if (s.incoherenceFlags & INCOH_BANGBANG_STEERING) os << "[BANGBANG_STEER] ";
        if (s.incoherenceFlags & INCOH_GRIP_COLLAPSE) os << "[GRIP_COLLAPSE] ";
        if (s.incoherenceFlags & INCOH_REVERSE_THRUST) os << "[REVERSE_THRUST] ";
        if (s.incoherenceFlags & INCOH_JACOBIAN_INVERTED) os << "[JACOBIAN_INVERTED] ";
        if (s.incoherenceFlags & INCOH_STALLING) os << "[STALLING] ";

        os << "\n";
        printed++;
        if (printed >= maxEntries) {
            os << "  ... [" << (samples.size() - printed) << " more ticks with anomalies omitted]\n";
            break;
        }
    }
    if (printed == 0) {
        os << "  [PERFECT RUN: ZERO COGNITIVE OR PHYSICAL INCOHERENCES DETECTED]\n";
    }
}
